#include "secrets.h"
#include "config.h"
#include <PubSubClient.h>
#include <TinyGsmClient.h>
#include <SSLClient.h>

#if TINY_GSM_USE_WIFI
#include <WiFi.h>
#include <WiFiClientSecure.h>

WiFiClientSecure wifiClientSecure = WiFiClientSecure();
#endif

TinyGsm modem(Serial_Sim7600);
TinyGsmClient gprsClient(modem); 
SSLClient gprsClientSecure(&gprsClient);
PubSubClient mqtt;

float lat = 0;
float lon = 0;
char Lat[20];
char Lon[20];
char sensorData[100];
char dateTime[30];
char sendbuffer[120];
uint32_t lastReconnectAttempt = 0;
unsigned long previousMillis = 0;

boolean mqttConnect() {
    Serial.print("Connecting to ");
    Serial.print(broker);
    Serial.print(":");
    Serial.println(brokerPort);

    boolean status = mqtt.connect("esp32_client");

    if (status == false) {
        Serial.print("MQTT connect failed, state: ");
        Serial.println(mqtt.state());
        return false;
    }
    Serial.println("success");
    mqtt.publish(configTopic, "esp32_client started");
    mqtt.subscribe(configTopic);
    return mqtt.connected();
}

void mqttCallback(char *topic, byte *payload, unsigned int len) {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.println("]: ");

    if (String(topic) == configTopic) {
      String content;
      for (unsigned int i = 0; i < len; i++) {
          content += (char)payload[i];
      }
      Serial.print("Content: ");
      Serial.print(content);

      if(content == "battery"){
        float batteryMv = readBattery();
        float batteryPercentage = getBatteryPercentage(batteryMv);

        Serial.printf("Battery: %.0fmV (%d%%)\n", batteryMv, (int)batteryPercentage);

        char batteryStr[20];
        snprintf(batteryStr, sizeof(batteryStr), "%.0fmV (%d%%)", batteryMv, (int)batteryPercentage);
        mqtt.publish(batteryTopic, batteryStr);
      }

    }
}

void mqttConnectAttempt() {
  static unsigned long lastReconnectAttempt = 0;

  unsigned long currentMillis = millis();
  if (currentMillis - lastReconnectAttempt > MQTT_RECONNECT_INTERVAL) {
    lastReconnectAttempt = currentMillis;
    if (mqttConnect()) {
      lastReconnectAttempt = 0;
    }
  }
}

void checkNetworkAndGPRS() {
  static unsigned long lastCheckMillis = 0;
  
  unsigned long currentMillis = millis();
  if (currentMillis - lastCheckMillis >= NETWORK_CHECK_INTERVAL) {
    lastCheckMillis = currentMillis;

    if (!modem.isNetworkConnected()) {
      Serial.println("Network disconnected");
      if (!modem.waitForNetwork(180000L, true)) {
        Serial.println("Network failed to reconnect");
        return;
      }
      Serial_Sim7600.println("Network re-connected");
    }

    if (!modem.isGprsConnected()) {
      Serial.println("GPRS disconnected!");
      Serial.print(F("Connecting to "));
      Serial.print(apn);
      if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        Serial.println("GPRS failed to reconnect");
        return;
      }
      Serial.println("GPRS reconnected");
    }
  }
}

#if TINY_GSM_USE_WIFI
void initWiFi() {
    Serial.println("Scanning for Wi-Fi networks...");
    int numNetworks = WiFi.scanNetworks();
    if (numNetworks == 0) {
        Serial.println("No networks found.");
    } else {
        Serial.print(numNetworks);
        Serial.println(" networks found:");
        for (int i = 0; i < numNetworks; ++i) {
            Serial.print("Network name: ");
            Serial.println(WiFi.SSID(i));
            Serial.println("-----------------------");
        }
    }
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifiSSID, wifiPass);
    Serial.print("Connecting to WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    wifiClientSecure.setCACert(AWS_CERT_CA);
    wifiClientSecure.setCertificate(AWS_CERT_CRT);
    wifiClientSecure.setPrivateKey(AWS_CERT_PRIVATE);
    Serial.println(" connected!");
}
#endif

void readSensorData() {
  if (Serial_Sensor.available() > 0) {
    int bytesRead = Serial_Sensor.readBytesUntil('\n', sensorData, sizeof(sensorData) - 1);
    if (bytesRead > 0) {
      sensorData[bytesRead] = '\0';
    } else {
      strcpy(sensorData, "NO_SENSOR_DATA");
      Serial.println("No sensor data available");
    }
  } else {
    strcpy(sensorData, "NO_SENSOR_DATA");
    Serial.println("No sensor data available");
  }
}

void readDateTime() {
  int year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0;
  float timezone = 0.0;
  if (modem.getNetworkTime(&year, &month, &day, &hour, &minute, &second, &timezone)) {
    snprintf(dateTime, sizeof(dateTime), "%04d-%02d-%02d %02d:%02d:%02d",
            year, month, day, hour, minute, second);
  } else {
    Serial.println("Failed to get network time.");
    strcpy(dateTime, "NO_DATETIME");
  }
}

void readGpsData() {
      modem.sendAT("+SGPIO=0,4,1,1");
      if (modem.waitResponse(10000L) != 1) {
        Serial.println(" SGPIO=0,4,1,1 false ");
      }
      modem.enableGPS();
      Serial.println("Requesting current GPS/GNSS/GLONASS location");
      if (modem.getGPS(&lat, &lon)) {
        Serial.println("Latitude: " + String(lat, 8) + "\tLongitude: " + String(lon, 8));
      } else {
        Serial.println("Failed to get GPS data.");
      }
}

void sendData() {
  readSensorData();
  readDateTime();
  readGpsData();

  int len = snprintf(sendbuffer, sizeof(sendbuffer),
                       "%.6f, %.6f, %s, %s",
                      lat, lon, dateTime, sensorData);

  if (len < 0 || len >= sizeof(sendbuffer)) {
    Serial.println("Error: snprintf failed or buffer overflow!");
    return;
  }

  Serial.print("Sending: ");
  Serial.println(sendbuffer);
  mqtt.publish(dataTopic, sendbuffer);
}

float getBatteryPercentage(float mv) {
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

float readBattery()
{
    int vref = 1100;
    uint16_t volt = analogRead(BAT_ADC);
    float battery_voltage = ((float)volt / 4095.0) * 2.0 * 3.3 * (vref);
    return battery_voltage;
}

void setup() {
    Serial.begin(UART_BAUD);
    delay(10);
    Serial_Sim7600.begin(UART_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
    delay(10);
    Serial_Sensor.begin(UART_SENSOR_BAUD, SERIAL_8N1, SENSOR_RX, SENSOR_TX);
 
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    pinMode(MODEM_PWRKEY, OUTPUT);
    digitalWrite(MODEM_PWRKEY, HIGH);
    delay(300);
    digitalWrite(MODEM_PWRKEY, LOW);
    pinMode(MODEM_FLIGHT, OUTPUT);
    digitalWrite(MODEM_FLIGHT, HIGH);

    Serial.println("Start modem...");

    for (int i = 0; i < 3; ++i) {
        while (!modem.testAT(5000)) {
            Serial.println("Try to start modem...");
            pinMode(MODEM_PWRKEY, OUTPUT);
            digitalWrite(MODEM_PWRKEY, HIGH);
            delay(300); //Need delay
            digitalWrite(MODEM_PWRKEY, LOW);
        }
    }
    Serial.println("Modem Response Started.");

    DBG("Initializing modem...");
    if (!modem.init()) {
        DBG("Failed to restart modem, delaying 10s and retrying");
    }

    String ret;
    ret = modem.setNetworkMode(2);
    DBG("setNetworkMode:", ret);
    String name = modem.getModemName();
    DBG("Modem Name:", name);
    String modemInfo = modem.getModemInfo();
    DBG("Modem Info:", modemInfo);

    if (GSM_PIN && modem.getSimStatus() != 3) {
        modem.simUnlock(GSM_PIN);
    }

    Serial.print("Waiting for GSM network...");
    if (!modem.waitForNetwork()) {
        Serial.println(" fail");
        delay(10000);
        return;
    }
    Serial.println(" success");

    if (modem.isNetworkConnected()) {
        Serial.println("GSM Network connected");
    }

    Serial.print(F("Connecting to "));
    Serial.print(apn);
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        Serial.println(" fail");
        delay(10000);
        return;
    }
    Serial.println(" success");

    gprsClientSecure.setCACert(AWS_CERT_CA);
    gprsClientSecure.setCertificate(AWS_CERT_CRT);
    gprsClientSecure.setPrivateKey(AWS_CERT_PRIVATE);

    if (modem.isGprsConnected()) {
        Serial.println("GPRS connected");
    }
    
    #if TINY_GSM_USE_WIFI
        initWiFi();
        mqtt.setClient(wifiClientSecure);
    #else
        mqtt.setClient(gprsClientSecure);
    #endif

    mqtt.setServer(broker, brokerPort);
    mqtt.setCallback(mqttCallback);
    mqttConnect();
}

void loop() {
  unsigned long currentMillis = millis();

  if (!modem.isNetworkConnected() || !modem.isGprsConnected()) {
    checkNetworkAndGPRS();
    return; 
  }

  if (!mqtt.connected()) {
    mqttConnectAttempt();
    return; 
  }

  mqtt.loop();

  if (currentMillis - previousMillis >= INTERVAL) {
    previousMillis = currentMillis;
    sendData();
  }
}