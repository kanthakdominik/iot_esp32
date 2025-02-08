#include "secrets.h"
#include "config.h"
#include <PubSubClient.h>
#include <TinyGsmClient.h>
#include <SSLClient.h>

TinyGsm modem(Serial_Sim7600);
TinyGsmClient gprsClient(modem); 
SSLClient gprsClientSecure(&gprsClient);
PubSubClient mqtt;

float lat = 0;
float lon = 0;
char Lat[20];
char Lon[20];
char sendbuffer[120];
char sensorData[100];
const char* Bat_value;
RTC_DATA_ATTR int bootCount = 0;
int ledStatus = LOW;
uint32_t lastReconnectAttempt = 0;
unsigned long previousMillis = 0;
const long interval = 1000; 

boolean mqttConnect() {
    Serial.print("Connecting to ");
    Serial.print(broker);
    Serial.print(":");
    Serial.println(brokerPort);

    // Connect to MQTT Broker
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
    Serial.print("]: ");
    Serial.write(payload, len);
    Serial.println();

    // Only proceed if incoming message's topic matches configTopic
    if (String(topic) == configTopic) {
        Serial.println("Set config");
    }
}

void mqttConnectAttempt() {
  static unsigned long lastReconnectAttempt = 0;
  const long reconnectInterval = 10000;

  unsigned long currentMillis = millis();
  if (currentMillis - lastReconnectAttempt > reconnectInterval) {
    lastReconnectAttempt = currentMillis;
    if (mqttConnect()) {
      lastReconnectAttempt = 0;
    }
  }
}

void checkNetworkAndGPRS() {
  static unsigned long lastCheckMillis = 0;
  const long checkInterval = 10000;

  unsigned long currentMillis = millis();
  if (currentMillis - lastCheckMillis >= checkInterval) {
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

void sendData() {
  // Read sensor data
  if (Serial_Sensor.available() > 0) {
    int bytesRead = Serial_Sensor.readBytesUntil('\n', sensorData, sizeof(sensorData) - 1);
    if (bytesRead > 0) {
      sensorData[bytesRead] = '\0'; // Null-terminate the string
      Serial.print("Received sensor data: ");
      Serial.println(sensorData);
    } else {
      strcpy(sensorData, "NO_SENSOR_DATA"); // Default value if no sensor data is available
      Serial.println("No sensor data available");
    }
  } else {
    strcpy(sensorData, "NO_SENSOR_DATA"); // Default value if no sensor data is available
    Serial.println("No sensor data available");
  }

  // Get the date and time from the SIM7600
  int year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0;
  float timezone = 0.0;
  char dateTime[30];
  if (modem.getNetworkTime(&year, &month, &day, &hour, &minute, &second, &timezone)) {
    Serial.print("Date: ");
    Serial.print(year);
    Serial.print("-");
    Serial.print(month);
    Serial.print("-");
    Serial.print(day);
    Serial.print(" Time: ");
    Serial.print(hour);
    Serial.print(":");
    Serial.print(minute);
    Serial.print(":");
    Serial.println(second);

    // Format the date and time into a string
    snprintf(dateTime, sizeof(dateTime), "%04d-%02d-%02d %02d:%02d:%02d",
            year, month, day, hour, minute, second);
  } else {
    Serial.println("Failed to get network time.");
    strcpy(dateTime, "NO_DATETIME");
  }

  // Get the GPS coordinates
    bool gpsDataValid = true;
    while (lat <= 0 || lon <= 0)
    {
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
        gpsDataValid = false;
        break;
      }
    }

  int len = snprintf(sendbuffer, sizeof(sendbuffer),
                       "%.6f,%.6f,%s,%s",
                      lat, lon, dateTime, sensorData);

  if (len < 0 || len >= sizeof(sendbuffer)) {
    Serial.println("Error: snprintf failed or buffer overflow!");
    return;
  }

  Serial.print("Sending: ");
  Serial.println(sendbuffer);
  mqtt.publish(dataTopic, sendbuffer);                     

  // mqtt.publish(dataTopic, "aaaaa");
}

void setup() {
    Serial.begin(UART_BAUD);
    delay(10);
    Serial_Sim7600.begin(UART_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
    delay(10);
    Serial_Sensor.begin(UART_SENSOR_BAUD, SERIAL_8N1, SENSOR_RX, SENSOR_TX);

 
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
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
    
    mqtt.setClient(gprsClientSecure);
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

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    sendData();
  }
}