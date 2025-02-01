#ifndef CONFIG_H
#define CONFIG_H

#define PRINT_INTERVAL      1000

#define TINY_GSM_MODEM_SIM7600
#define TINY_GSM_RX_BUFFER  1024        /* Set RX buffer to 1Kb */

#define Serial_Sim7600      Serial1
#define Serial_Sensor       Serial2

#define uS_TO_S_FACTOR      1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP       30          /* Time ESP32 will go to sleep (in seconds) */

#define UART_BAUD           115200
#define UART_SENSOR_BAUD    9600

#define MODEM_TX            27
#define MODEM_RX            26
#define MODEM_PWRKEY        4
#define MODEM_DTR           32
#define MODEM_RI            33
#define MODEM_FLIGHT        25
#define MODEM_STATUS        34

#define SENSOR_RX           14
#define SENSOR_TX           13

#endif // CONFIG_H