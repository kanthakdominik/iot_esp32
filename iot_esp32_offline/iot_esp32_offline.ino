#include "config.h"
#include <Preferences.h>
#include <SD.h>

char sensorData[100];
char writeBuffer[120];
char filename[50];

Preferences preferences;
uint32_t currentRouteId = 0;
unsigned long previousMillis = 0;

void initRouteId() {
  preferences.begin("routes", false);  // false = R/W mode
  currentRouteId = preferences.getUInt("lastRoute", 0);
  currentRouteId++;
  preferences.putUInt("lastRoute", currentRouteId);
  preferences.end();
  
  Serial.print("Starting new route with ID: ");
  Serial.println(currentRouteId);
}

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

void writeData() {
  readSensorData();

  int len = snprintf(writeBuffer, sizeof(writeBuffer),
                       "%u, %s",
                        currentRouteId, sensorData);

  Serial.print("Writing: ");
  Serial.println(writeBuffer);

  File dataFile = SD.open(filename, FILE_APPEND);
  if (dataFile) {
    dataFile.println(writeBuffer);
    dataFile.close();
    Serial.println("Data written to SD card.");
  } else {
    Serial.println("Failed to open file on SD card.");
  }
}

void setup() {
    Serial.begin(UART_BAUD);
    delay(10);
    Serial_Sensor.begin(UART_SENSOR_BAUD, SERIAL_8N1, SENSOR_RX, SENSOR_TX);
 
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    initRouteId();

    snprintf(filename, sizeof(filename), "/route_%u.txt", currentRouteId);

    if (!SD.begin(SD_CS)) {
        Serial.println("SD card initialization failed!");
    } else {
        Serial.println("SD card initialized.");
        Serial.println(filename);
    }
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= INTERVAL) {
    previousMillis = currentMillis;
    writeData();
  }
}