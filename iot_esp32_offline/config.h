#ifndef CONFIG_H
#define CONFIG_H

#define INTERVAL                1000

#define Serial_Sensor       Serial2

#define uS_TO_S_FACTOR      1000000ULL

#define UART_BAUD           115200
#define UART_SENSOR_BAUD    9600

#define SD_MISO             2
#define SD_MOSI             15
#define SD_SCLK             14
#define SD_CS               13

#define LED_PIN             12

#define SENSOR_RX           16
#define SENSOR_TX           17

#endif // CONFIG_H