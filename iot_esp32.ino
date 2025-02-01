#include "secrets.h"
#include "config.h"
#include <ArduinoJson.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <TinyGsmClient.h>
#include <SSLClient.h>
#include <TinyGPS++.h>


// Clients declarations
TinyGPSPlus gps;
TinyGsm modem(Serial_Sim7600);
TinyGsmClient gprsClient(modem); 
SSLClient gprsClientSecure(&gprsClient);
WiFiClientSecure wifiClientSecure = WiFiClientSecure();
PubSubClient mqtt;

static uint32_t lastPrintTime = 0;
uint32_t lastReconnectAttempt = 0;
uint32_t lastMessageTime = 0;
String lastGpsLocation = "";
String lastGpsDateTime = "";
String lastSensorData = "";
bool newGpsDataAvailable = false;
bool newSensorDataAvailable = false;

void init() {
    Serial.begin(UART_BAUD);
    delay(10);
    Serial_Sim7600.begin(UART_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
    delay(10);
    Serial_Sensor.begin(UART_SENSOR_BAUD, SERIAL_8N1, SENSOR_RX, SENSOR_TX);
}

void initWiFi() {
    // Scan for available Wi-Fi networks
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

    // Connect to Wi-Fi
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

void initGPRS() {
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
}

void initSim7600() {
    /*
    MODEM_PWRKEY IO:4 The power-on signal of the modulator must be given to it,
    otherwise the modulator will not reply when the command is sent
    */
    pinMode(MODEM_PWRKEY, OUTPUT);
    digitalWrite(MODEM_PWRKEY, HIGH);
    delay(300); //Need delay
    digitalWrite(MODEM_PWRKEY, LOW);

    /*
    MODEM_FLIGHT IO:25 Modulator flight mode control,
    need to enable modulator, this pin must be set to high
    */
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
}

void initGps() {
    // Stop GPS Server
    modem.sendAT("+CGPS=0");
    modem.waitResponse(30000);
    // Configure GNSS support mode
    modem.sendAT("+CGNSSMODE=15,1");
    modem.waitResponse(30000);
    // Configure NMEA sentence type
    modem.sendAT("+CGPSNMEA=200191");
    modem.waitResponse(30000);
    // Set NMEA output rate to 1HZ
    modem.sendAT("+CGPSNMEARATE=1");
    modem.waitResponse(30000);
    // Enable GPS
    modem.sendAT("+CGPS=1");
    modem.waitResponse(30000);
    // Download Report GPS NMEA-0183 sentence , NMEA TO AT PORT
    modem.sendAT("+CGPSINFOCFG=1,31");
    modem.waitResponse(30000);
}

void initMQTT() {
    #if TINY_GSM_USE_WIFI
        mqtt.setClient(wifiClientSecure);
    #else
        mqtt.setClient(gprsClientSecure);
    #endif
    mqtt.setServer(broker, brokerPort);
    mqtt.setCallback(mqttCallback);
    mqttConnect();
}

void checkMQTTConnection() {
    if (!mqtt.connected()) {
        Serial.println("=== MQTT NOT CONNECTED ===");
        uint32_t t = millis();
        if (t - lastReconnectAttempt > 10000L) {
            lastReconnectAttempt = t;
            if (mqttConnect()) {
                lastReconnectAttempt = 0;
            }
        }
        delay(1000);
        return;
    }
    mqtt.loop();
}

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

const char* getDataAsJson() {
    StaticJsonDocument<200> doc;
    static char jsonBuffer[512];

    doc["location"] = lastGpsLocation;
    doc["dateTime"] = lastGpsDateTime;
    doc["sensorData"] = lastSensorData;
    serializeJson(doc, jsonBuffer);
    return jsonBuffer;
}

void sendData() {
    uint32_t currentTime = millis();
    if ((currentTime - lastMessageTime >= 1000) && newGpsDataAvailable && newSensorDataAvailable) {
        mqtt.publish(dataTopic, getDataAsJson());
        lastMessageTime = currentTime;
        newGpsDataAvailable = false;
        newSensorDataAvailable = false;
    }
}

void processGPSData() {
    // Read all available GPS data
    while (Serial_Sim7600.available()) {
        char c = Serial_Sim7600.read();
        if (gps.encode(c)) {
          if (millis() - lastPrintTime >= PRINT_INTERVAL) {
            String gpsLocation = getGpsLocation();
            Serial.println("Location: " + gpsLocation);
            String gpsDateTime = getGpsDateTime();
            Serial.println("DateTime: " + gpsDateTime);
            
            // Store for next MQTT publish
            lastGpsLocation = gpsLocation;
            lastGpsDateTime = gpsDateTime;
            newGpsDataAvailable = true;
            lastPrintTime = millis();
          }
        }
    }
}

void processSensorData() {
    // Read all available sensor data
    while (Serial_Sensor.available()) {
        String sensorData = Serial_Sensor.readStringUntil('\n');
        sensorData.trim();
        Serial.println("Sensor data: " + sensorData);

        // Store for next MQTT publish
        lastSensorData = sensorData;
        newSensorDataAvailable = true;
    }
}

String getGpsLocation() {
    if(gps.location.isValid()) {
        String gpsLocation = String(gps.location.lat(), 6);
        gpsLocation += ",";
        gpsLocation += String(gps.location.lng(), 6);
        return gpsLocation; 
    } else return "INVALID";
}

String getGpsDateTime() {
    if(gps.date.isValid() && gps.time.isValid()) {
        String gpsDateTime = "";
        if (gps.date.month() < 10) gpsDateTime += "0";
        gpsDateTime += String(gps.date.month());
        gpsDateTime += "/";
        if (gps.date.day() < 10) gpsDateTime += "0";
        gpsDateTime += String(gps.date.day());
        gpsDateTime += "/";
        gpsDateTime += String(gps.date.year());
        gpsDateTime += " ";
        if (gps.time.hour() < 10) gpsDateTime += "0";
        gpsDateTime += String(gps.time.hour());
        gpsDateTime += ":";
        if (gps.time.minute() < 10) gpsDateTime += "0";
        gpsDateTime += String(gps.time.minute());
        gpsDateTime += ":";
        if (gps.time.second() < 10) gpsDateTime += "0";
        gpsDateTime += String(gps.time.second());
        gpsDateTime += ".";
        if (gps.time.centisecond() < 10) gpsDateTime += "0";
        gpsDateTime += String(gps.time.centisecond());
        return gpsDateTime; 
    } else return "INVALID";
}

void setup() {
    init();
    #if TINY_GSM_USE_WIFI
        initWiFi();
    #else
        initGPRS();
    #endif
    initSim7600();
    initGps();
    initMQTT();
}

void loop() {
    checkMQTTConnection();
    processGPSData();
    processSensorData();
    sendData();
}