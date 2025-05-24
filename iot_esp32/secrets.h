#ifndef SECRETS_H
#define SECRETS_H

#include <pgmspace.h>


// WiFi connection credentials
const char wifiSSID[] = "";
const char wifiPass[] = "";

// GPRS credentials
const char apn[]  = "";
const char gprsUser[] = "";
const char gprsPass[] = "";

// GSM PIN
#define GSM_PIN ""

// MQTT details
const char *broker = "";
const int brokerPort = ;
const char *dataTopic = "";
const char *configTopic = "";
const char *batteryTopic = "";

// Amazon Root CA 1
static const char AWS_CERT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
-----END CERTIFICATE-----
)EOF";

// Device Certificate
static const char AWS_CERT_CRT[] PROGMEM = R"KEY(
-----BEGIN CERTIFICATE-----
-----END CERTIFICATE-----
)KEY";

// Device Private Key
static const char AWS_CERT_PRIVATE[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----
-----END RSA PRIVATE KEY-----
)KEY";

#endif // SECRETS_H