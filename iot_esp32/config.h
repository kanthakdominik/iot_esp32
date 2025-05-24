#ifndef CONFIG_H
#define CONFIG_H

#define INTERVAL                1000
#define MQTT_RECONNECT_INTERVAL 10000
#define NETWORK_CHECK_INTERVAL  10000

#define ESP_DRD_USE_EEPROM    false
#define ESP_DRD_USE_SPIFFS    true
#define DRD_TIMEOUT           5
#define DRD_ADDRESS           0

#define SD_MISO             2
#define SD_MOSI             15
#define SD_SCLK             14
#define SD_CS               13

#define TINY_GSM_MODEM_SIM7600
#define TINY_GSM_RX_BUFFER  1024

#define Serial_Sim7600      Serial1
#define Serial_Sensor       Serial2

#define uS_TO_S_FACTOR      1000000ULL

#define UART_BAUD           115200
#define UART_SENSOR_BAUD    9600

#define MODEM_TX            27
#define MODEM_RX            26
#define MODEM_PWRKEY        4
#define MODEM_DTR           32
#define MODEM_RI            33
#define MODEM_FLIGHT        25
#define MODEM_STATUS        34

#define LED_PIN             12
#define BAT_ADC             35

#define SENSOR_RX           39
#define SENSOR_TX           0

#define TINY_GSM_USE_WIFI   false

#endif // CONFIG_H