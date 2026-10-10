// ============================================================================
// Mobile gamma radiation logger - LILYGO T-SIM7600G-H (ESP32) + MightyOhm Geiger counter
//
// Protocol: MQTT v1, see iot_spring/docs/PROTOCOL.md
//   radmon/v1/<deviceId>/telemetry  one JSON sample per MightyOhm line (1 Hz)
//   radmon/v1/<deviceId>/status     retained {"online":true,...,"csq":N}; Last Will {"online":false}
//   radmon/v1/<deviceId>/event      battery (also every HEALTH_INTERVAL), route-started, error
//   radmon/v1/<deviceId>/cmd        {"cmd":"new-route"} / {"cmd":"battery"}
//
// Routes:
//   power on / RST button   -> new route (r + 1, seq = 0)
//   "new-route" command     -> new route without reboot (modem and GPS fix are kept)
//   brownout / crash / WDT  -> SAME route, seq continues
//
// Required libraries (Arduino Library Manager):
//   TinyGSM (Volodymyr Shymanskyy), SSLClient (govorox), PubSubClient (Nick O'Leary),
//   ArduinoJson (Benoit Blanchon, v7)
// ============================================================================

#include "config.h"
#include <Preferences.h>
#include <SD.h>
#include <SPI.h>
#include <sys/time.h>
#include <time.h>
#include <esp_system.h>
#include "secrets.h"
#include <PubSubClient.h>
#include <TinyGsmClient.h>
#include <SSLClient.h>
#include <ArduinoJson.h>

#if USE_WIFI
#include <WiFi.h>
#include <WiFiClientSecure.h>
WiFiClientSecure wifiClientSecure;
#endif

// ---------------------------------------------------------------------------- networking
TinyGsm modem(Serial_Sim7600);
TinyGsmClient gprsClient(modem);
SSLClient gprsClientSecure(&gprsClient);
PubSubClient mqtt;

char topicTelemetry[64];
char topicStatus[64];
char topicEvent[64];
char topicCmd[64];
const char *LWT_PAYLOAD = "{\"online\":false}";

bool networkOk = false;
unsigned long lastNetworkCheck = 0;
unsigned long lastMqttAttempt = 0;
unsigned long lastHealthPublish = 0;

// ---------------------------------------------------------------------------- route state
// Survives software resets, panics and watchdog resets (not power-on).
#define RTC_MAGIC 0x52414431UL  // "RAD1"
RTC_NOINIT_ATTR uint32_t rtcMagic;
RTC_NOINIT_ATTR uint32_t rtcRouteNo;
RTC_NOINIT_ATTR uint32_t rtcSeq;

Preferences prefs;
uint32_t routeNo = 0;
uint32_t seq = 0;
const char *resetReasonName = "UNKNOWN";

// ---------------------------------------------------------------------------- sensor
char sensorLine[96];
size_t sensorLineLen = 0;
unsigned long lastSensorLineAt = 0;
bool sensorTimeoutReported = false;

struct SensorReading {
  int cps;
  int cpm;
  float usv;
  char mode[8];
};

// ---------------------------------------------------------------------------- GNSS / clock
enum ClockSource { CLOCK_NONE, CLOCK_NET, CLOCK_GNSS };
ClockSource clockSource = CLOCK_NONE;
unsigned long lastClockSync = 0;

struct GnssReading {
  bool fix;
  float lat, lon, speed, hdop;
  int sats;
};

// ---------------------------------------------------------------------------- SD card
bool sdReady = false;

// ---------------------------------------------------------------------------- forward declarations
void publishStatus();
void publishBattery();
bool readSensor(SensorReading &out);
void takeSample(const SensorReading &s);
GnssReading readGnss();

// ---------------------------------------------------------------------------- LED
bool validGpsAcquired = false;
unsigned long lastLedToggle = 0;
int ledBlinksPending = 0;            // quick blinks after a new route
unsigned long ledPatternStep = 0;

// ============================================================================ helpers

const char *resetReasonToString(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT";
    case ESP_RST_SW:        return "SW";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "UNKNOWN";
  }
}

// Days since 1970-01-01 for a civil date (Howard Hinnant's algorithm) - timezone independent.
long daysFromCivil(int y, unsigned m, unsigned d) {
  y -= m <= 2;
  const long era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = (unsigned)(y - era * 400);
  const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + (long)doe - 719468;
}

time_t utcToEpoch(int year, int month, int day, int hour, int minute, int second) {
  return (time_t)daysFromCivil(year, month, day) * 86400L + hour * 3600L + minute * 60L + second;
}

void setSystemClock(time_t epoch, ClockSource source) {
  struct timeval tv = { epoch, 0 };
  settimeofday(&tv, nullptr);
  clockSource = source;
  lastClockSync = millis();
}

// "2026-10-08T19:56:03Z" or "" when the clock was never set
void formatTimestamp(char *out, size_t len) {
  if (clockSource == CLOCK_NONE) {
    out[0] = '\0';
    return;
  }
  time_t now = time(nullptr);
  struct tm t;
  gmtime_r(&now, &t);
  strftime(out, len, "%Y-%m-%dT%H:%M:%SZ", &t);
}

float readBatteryMv() {
  // the battery voltage is divided by 2 on the board
  return analogReadMilliVolts(BAT_ADC) * 2.0f;
}

int batteryPercent(float mv) {
  if (mv >= 4200) return 100;
  if (mv >= 4100) return 90;
  if (mv >= 4000) return 80;
  if (mv >= 3900) return 70;
  if (mv >= 3800) return 60;
  if (mv >= 3750) return 50;
  if (mv >= 3700) return 40;
  if (mv >= 3600) return 30;
  if (mv >= 3500) return 20;
  if (mv >= 3400) return 10;
  if (mv >= 3300) return 5;
  return 0;
}

// GSM signal quality (AT+CSQ) 0..31, or -1 when unknown (modem answers 99) or when sending over Wi-Fi.
int readSignalQuality() {
#if USE_WIFI
  return -1;
#else
  int q = modem.getSignalQuality();
  return (q >= 0 && q <= 31) ? q : -1;
#endif
}

// ',"csq":N' or "" when the signal quality is unknown
void formatCsqPart(char *out, size_t len) {
  int csq = readSignalQuality();
  if (csq < 0) {
    out[0] = '\0';
  } else {
    snprintf(out, len, ",\"csq\":%d", csq);
  }
}

// ============================================================================ route state

void persistRoute() {
  prefs.begin("radmon", false);
  prefs.putUInt("route", routeNo);
  prefs.putUInt("seqck", seq);
  prefs.end();
  rtcMagic = RTC_MAGIC;
  rtcRouteNo = routeNo;
  rtcSeq = seq;
}

void initRouteState() {
  esp_reset_reason_t reason = esp_reset_reason();
  resetReasonName = resetReasonToString(reason);
  bool intentionalStart = reason == ESP_RST_POWERON || reason == ESP_RST_EXT || reason == ESP_RST_UNKNOWN;

  prefs.begin("radmon", false);
  uint32_t lastRoute = prefs.getUInt("route", 0);
  uint32_t seqCheckpoint = prefs.getUInt("seqck", 0);
  prefs.end();

  if (lastRoute == 0) {
    // migrate the counter of firmware 1.x (namespace "routes", key "lastRoute")
    prefs.begin("routes", true);
    lastRoute = prefs.getUInt("lastRoute", 0);
    prefs.end();
    intentionalStart = true;
  }

  if (intentionalStart) {
    routeNo = lastRoute + 1;
    seq = 0;
  } else {
    routeNo = lastRoute;
    if (rtcMagic == RTC_MAGIC && rtcRouteNo == routeNo) {
      seq = rtcSeq;                                   // exact value from RTC memory
    } else {
      seq = seqCheckpoint + SEQ_CHECKPOINT_EVERY;     // safe value above anything already sent
    }
  }
  persistRoute();

  Serial.printf("Reset reason: %s -> %s route %u, seq %u\n", resetReasonName,
                intentionalStart ? "NEW" : "CONTINUE", (unsigned)routeNo, (unsigned)seq);
}

void startNewRoute(const char *reason) {
  routeNo++;
  seq = 0;
  persistRoute();
  ledBlinksPending = 3;
  Serial.printf("New route %u (%s)\n", (unsigned)routeNo, reason);

  if (mqtt.connected()) {
    char buf[96];
    snprintf(buf, sizeof(buf), "{\"type\":\"route-started\",\"r\":%u,\"reason\":\"%s\"}", (unsigned)routeNo, reason);
    mqtt.publish(topicEvent, buf);
    publishStatus();
  }
}

void advanceSeq() {
  seq++;
  rtcSeq = seq;
  if (seq % SEQ_CHECKPOINT_EVERY == 0) {
    prefs.begin("radmon", false);
    prefs.putUInt("seqck", seq);
    prefs.end();
  }
}

// ============================================================================ sensor

// Reads characters without blocking; returns true when a complete, valid line was parsed.
bool readSensor(SensorReading &out) {
  while (Serial_Sensor.available() > 0) {
    char c = (char)Serial_Sensor.read();
    if (c == '\r') continue;
    if (c != '\n') {
      if (sensorLineLen < sizeof(sensorLine) - 1) sensorLine[sensorLineLen++] = c;
      continue;
    }
    sensorLine[sensorLineLen] = '\0';
    sensorLineLen = 0;

    // MightyOhm format: "CPS, 0, CPM, 18, uSv/hr, 0.10, SLOW"
    SensorReading r;
    if (sscanf(sensorLine, "CPS, %d, CPM, %d, uSv/hr, %f, %7[A-Z]", &r.cps, &r.cpm, &r.usv, r.mode) == 4) {
      lastSensorLineAt = millis();
      sensorTimeoutReported = false;
      out = r;
      return true;
    }
    Serial.printf("Unrecognised sensor line: %s\n", sensorLine);
  }
  return false;
}

void checkSensorTimeout() {
  if (sensorTimeoutReported || millis() - lastSensorLineAt < SENSOR_TIMEOUT) return;
  sensorTimeoutReported = true;
  Serial.println("No data from MightyOhm");
  if (mqtt.connected()) {
    mqtt.publish(topicEvent, "{\"type\":\"error\",\"code\":\"sensor-timeout\",\"msg\":\"no data from MightyOhm\"}");
  }
}

// ============================================================================ GNSS

void initGnss() {
  modem.sendAT("+SGPIO=0,4,1,1");      // power the active GPS antenna
  if (modem.waitResponse(10000L) != 1) {
    Serial.println("SGPIO=0,4,1,1 failed");
  }
  if (!modem.enableGPS()) {
    Serial.println("enableGPS failed");
  }
}

GnssReading readGnss() {
  GnssReading g = { false, 0, 0, 0, 0, 0 };
  float alt = 0;
  int vsat = 0, usat = 0;
  int year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0;

  if (modem.getGPS(&g.lat, &g.lon, &g.speed, &alt, &vsat, &usat, &g.hdop,
                   &year, &month, &day, &hour, &minute, &second)) {
    g.fix = !(g.lat == 0.0f && g.lon == 0.0f);
    g.sats = usat > 0 ? usat : vsat;

    // GNSS time is UTC - the most accurate clock we have
    bool syncDue = clockSource != CLOCK_GNSS || millis() - lastClockSync > CLOCK_SYNC_INTERVAL;
    if (g.fix && year >= 2024 && syncDue) {
      setSystemClock(utcToEpoch(year, month, day, hour, minute, second), CLOCK_GNSS);
    }
  }
  validGpsAcquired = g.fix;
  return g;
}

// Fallback clock from the mobile network (local time + timezone -> UTC), used until the first GNSS fix.
void syncClockFromNetwork() {
  int year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0;
  float timezone = 0;
  if (modem.getNetworkTime(&year, &month, &day, &hour, &minute, &second, &timezone) && year >= 2024) {
    time_t local = utcToEpoch(year, month, day, hour, minute, second);
    setSystemClock(local - (time_t)(timezone * 3600.0f), CLOCK_NET);
    Serial.printf("Clock set from network (tz %+.2f h)\n", timezone);
  }
}

// ============================================================================ SD card

void initSd() {
#if SD_LOGGING
  SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
  sdReady = SD.begin(SD_CS);
  Serial.println(sdReady ? "SD card ready" : "No SD card - local logging disabled");
#endif
}

void logToSd(const char *json) {
#if SD_LOGGING
  if (!sdReady) return;
  char path[24];
  snprintf(path, sizeof(path), "/r%u.jsonl", (unsigned)routeNo);
  File f = SD.open(path, FILE_APPEND);
  if (!f) {
    Serial.println("SD write failed");
    return;
  }
  f.println(json);
  f.close();
#endif
}

// ============================================================================ MQTT

void publishStatus() {
  float mv = readBatteryMv();
  char csqPart[16];
  formatCsqPart(csqPart, sizeof(csqPart));
  char buf[192];
  snprintf(buf, sizeof(buf),
           "{\"online\":true,\"fw\":\"%s\",\"r\":%u,\"reset\":\"%s\",\"bat_mv\":%d,\"bat_pct\":%d%s}",
           FW_VERSION, (unsigned)routeNo, resetReasonName, (int)mv, batteryPercent(mv), csqPart);
  mqtt.publish(topicStatus, buf, true);   // retained
}

// Battery (+ GSM signal) event: answer to the "battery" command and periodic health report.
void publishBattery() {
  float mv = readBatteryMv();
  char csqPart[16];
  formatCsqPart(csqPart, sizeof(csqPart));
  char buf[96];
  snprintf(buf, sizeof(buf), "{\"type\":\"battery\",\"mv\":%d,\"pct\":%d%s}",
           (int)mv, batteryPercent(mv), csqPart);
  mqtt.publish(topicEvent, buf);
  lastHealthPublish = millis();
  Serial.printf("Battery: %s\n", buf);
}

// Every HEALTH_INTERVAL while connected: battery level and signal quality for the device history.
void maintainHealthReport() {
  if (mqtt.connected() && millis() - lastHealthPublish >= HEALTH_INTERVAL) {
    publishBattery();
  }
}

void mqttCallback(char *topic, byte *payload, unsigned int len) {
  if (strcmp(topic, topicCmd) != 0) return;

  JsonDocument doc;
  if (deserializeJson(doc, payload, len)) {
    Serial.println("Invalid command payload");
    return;
  }
  const char *cmd = doc["cmd"] | "";
  Serial.printf("Command: %s\n", cmd);

  if (strcmp(cmd, "new-route") == 0) {
    startNewRoute("cmd");
  } else if (strcmp(cmd, "battery") == 0) {
    publishBattery();
  } else {
    Serial.println("Unknown command");
  }
}

bool mqttConnect() {
  Serial.printf("MQTT connecting to %s:%d as %s ... ", broker, brokerPort, deviceId);
  // Last Will: the broker publishes {"online":false} (retained) when the connection is lost
  if (!mqtt.connect(deviceId, nullptr, nullptr, topicStatus, 1, true, LWT_PAYLOAD)) {
    Serial.printf("failed, state %d\n", mqtt.state());
    return false;
  }
  Serial.println("connected");
  mqtt.subscribe(topicCmd, 1);
  publishStatus();
  lastHealthPublish = millis();   // the status already carries battery and signal
  return true;
}

// Never blocks for long: measurements and SD logging must continue while offline.
void maintainConnectivity() {
  unsigned long now = millis();

#if !USE_WIFI
  if (!mqtt.connected() && now - lastNetworkCheck >= NETWORK_CHECK_INTERVAL) {
    lastNetworkCheck = now;
    networkOk = modem.isNetworkConnected() || modem.waitForNetwork(NETWORK_WAIT_TIMEOUT, true);
    if (networkOk && !modem.isGprsConnected()) {
      Serial.print("GPRS connecting ... ");
      networkOk = modem.gprsConnect(apn, gprsUser, gprsPass);
      Serial.println(networkOk ? "ok" : "failed");
    }
    if (networkOk && clockSource == CLOCK_NONE) {
      syncClockFromNetwork();
    }
  }
#else
  networkOk = WiFi.status() == WL_CONNECTED;
#endif

  if (networkOk && !mqtt.connected() && now - lastMqttAttempt >= MQTT_RECONNECT_INTERVAL) {
    lastMqttAttempt = now;
    mqttConnect();
  }
  mqtt.loop();
}

// ============================================================================ sample

void takeSample(const SensorReading &s) {
  GnssReading g = readGnss();

  char ts[24];
  formatTimestamp(ts, sizeof(ts));
  char tsPart[64] = "";
  if (ts[0] != '\0') {
    snprintf(tsPart, sizeof(tsPart), "\"ts\":\"%s\",\"tsrc\":\"%s\",", ts,
             clockSource == CLOCK_GNSS ? "gnss" : "net");
  }
  char posPart[128] = "";
  if (g.fix) {
    snprintf(posPart, sizeof(posPart), "\"lat\":%.6f,\"lon\":%.6f,\"sats\":%d,\"hdop\":%.1f,\"spd\":%.1f,",
             g.lat, g.lon, g.sats, g.hdop, g.speed);
  }

  char json[384];
  int n = snprintf(json, sizeof(json),
                   "{\"v\":1,\"r\":%u,\"seq\":%u,%s\"fix\":%s,%s\"cps\":%d,\"cpm\":%d,\"usv\":%.2f,\"mode\":\"%s\"}",
                   (unsigned)routeNo, (unsigned)seq, tsPart, g.fix ? "true" : "false", posPart,
                   s.cps, s.cpm, s.usv, s.mode);
  if (n < 0 || n >= (int)sizeof(json)) {
    Serial.println("Telemetry buffer overflow");
    return;
  }

  Serial.println(json);
  logToSd(json);
  if (mqtt.connected()) {
    mqtt.publish(topicTelemetry, json);
  }
  advanceSeq();
}

// ============================================================================ LED

void updateLed() {
  unsigned long now = millis();
  if (ledBlinksPending > 0) {                        // 3 quick blinks = new route accepted
    if (now - ledPatternStep >= 100) {
      ledPatternStep = now;
      bool on = !digitalRead(LED_PIN);
      digitalWrite(LED_PIN, on);
      if (!on) ledBlinksPending--;
    }
    return;
  }
  if (validGpsAcquired) {
    digitalWrite(LED_PIN, HIGH);                     // steady = GNSS fix
  } else if (now - lastLedToggle >= 500) {           // blinking = waiting for fix
    lastLedToggle = now;
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}

// ============================================================================ modem

bool startModem() {
  pinMode(MODEM_PWRKEY, OUTPUT);
  pinMode(MODEM_FLIGHT, OUTPUT);
  digitalWrite(MODEM_FLIGHT, HIGH);

  for (int attempt = 0; attempt < 10; attempt++) {
    if (modem.testAT(5000)) {
      Serial.println("Modem responding");
      modem.init();
      modem.setNetworkMode(2);   // automatic (LTE / 3G / 2G)
      if (strlen(GSM_PIN) > 0 && modem.getSimStatus() != 3) {
        modem.simUnlock(GSM_PIN);
      }
      Serial.printf("Modem: %s\n", modem.getModemInfo().c_str());
      return true;
    }
    Serial.println("Powering modem on ...");
    digitalWrite(MODEM_PWRKEY, HIGH);
    delay(300);
    digitalWrite(MODEM_PWRKEY, LOW);
  }
  Serial.println("Modem not responding - working offline (SD only)");
  return false;
}

// ============================================================================ setup / loop

void setup() {
  Serial.begin(UART_BAUD);
  delay(100);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.printf("\nRadiation logger fw %s, device %s\n", FW_VERSION, deviceId);

  snprintf(topicTelemetry, sizeof(topicTelemetry), "radmon/v1/%s/telemetry", deviceId);
  snprintf(topicStatus, sizeof(topicStatus), "radmon/v1/%s/status", deviceId);
  snprintf(topicEvent, sizeof(topicEvent), "radmon/v1/%s/event", deviceId);
  snprintf(topicCmd, sizeof(topicCmd), "radmon/v1/%s/cmd", deviceId);

  initRouteState();

  Serial_Sensor.setRxBufferSize(1024);   // keeps samples while the modem blocks for a few seconds
  Serial_Sensor.begin(UART_SENSOR_BAUD, SERIAL_8N1, SENSOR_RX, SENSOR_TX);
  lastSensorLineAt = millis();

  initSd();

  Serial_Sim7600.begin(UART_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  if (startModem()) {
    initGnss();
  }

#if USE_WIFI
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSSID, wifiPass);
  wifiClientSecure.setCACert(AWS_CERT_CA);
  wifiClientSecure.setCertificate(AWS_CERT_CRT);
  wifiClientSecure.setPrivateKey(AWS_CERT_PRIVATE);
  mqtt.setClient(wifiClientSecure);
#else
  gprsClientSecure.setCACert(AWS_CERT_CA);
  gprsClientSecure.setCertificate(AWS_CERT_CRT);
  gprsClientSecure.setPrivateKey(AWS_CERT_PRIVATE);
  mqtt.setClient(gprsClientSecure);
#endif

  mqtt.setServer(broker, brokerPort);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(1024);
  mqtt.setKeepAlive(60);
  mqtt.setSocketTimeout(15);

  // first connection attempt right away
  lastNetworkCheck = millis() - NETWORK_CHECK_INTERVAL;
  lastMqttAttempt = millis() - MQTT_RECONNECT_INTERVAL;
}

void loop() {
  SensorReading reading;
  if (readSensor(reading)) {
    takeSample(reading);          // one sample per MightyOhm line (1 Hz)
  }
  checkSensorTimeout();
  maintainConnectivity();
  maintainHealthReport();
  updateLed();
}
