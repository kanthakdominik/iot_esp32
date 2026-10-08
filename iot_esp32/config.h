#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// Firmware configuration (no secrets here - those live in secrets.h)
// Protocol: iot_spring/docs/PROTOCOL.md (MQTT v1)
// ============================================================================

#define FW_VERSION              "2.0.0"

// ---- timing ----------------------------------------------------------------
#define NETWORK_CHECK_INTERVAL  10000UL   // ms between GSM/GPRS checks when not connected
#define MQTT_RECONNECT_INTERVAL 10000UL   // ms between MQTT reconnect attempts
#define NETWORK_WAIT_TIMEOUT    5000UL    // ms - short, so measurements continue while offline
#define CLOCK_SYNC_INTERVAL     60000UL   // ms between system clock syncs from GNSS
#define SENSOR_TIMEOUT          5000UL    // ms without a line from MightyOhm -> error event
#define SEQ_CHECKPOINT_EVERY    300       // samples between seq checkpoints in NVS (flash wear)

// ---- features ----------------------------------------------------------------
#define SD_LOGGING              1         // 1 = every sample is also appended to /r<route>.jsonl on the SD card
#define USE_WIFI                0         // 1 = send over Wi-Fi instead of LTE (GNSS still from the modem)

// ---- SD card (SPI) -------------------------------------------------------------
#define SD_MISO             2
#define SD_MOSI             15
#define SD_SCLK             14
#define SD_CS               13

// ---- modem SIM7600G-H ------------------------------------------------------------
#define TINY_GSM_MODEM_SIM7600
#define TINY_GSM_RX_BUFFER  1024

#define Serial_Sim7600      Serial1
#define Serial_Sensor       Serial2

#define UART_BAUD           115200
#define UART_SENSOR_BAUD    9600

#define MODEM_TX            27
#define MODEM_RX            26
#define MODEM_PWRKEY        4
#define MODEM_DTR           32
#define MODEM_RI            33
#define MODEM_FLIGHT        25
#define MODEM_STATUS        34

// ---- board ---------------------------------------------------------------------
#define LED_PIN             12
#define BAT_ADC             35

// ---- MightyOhm Geiger counter (UART 9600 8N1, read only) ---------------------------
#define SENSOR_RX           39
// The ESP32 never writes to the counter, so TX is not used. This frees IO0
// (a boot strapping pin) for a future user button.
#define SENSOR_TX           -1

#endif // CONFIG_H
