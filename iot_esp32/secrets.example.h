#ifndef SECRETS_H
#define SECRETS_H

// ============================================================================
// Copy this file to secrets.h and fill it in. secrets.h is in .gitignore.
// The certificate block can be pasted from deploy/certs/<deviceId>/secrets_certs.h
// generated on the server by deploy/scripts/gen-certs.sh <deviceId>.
// ============================================================================

#include <pgmspace.h>

// Device identity - MUST equal the CN of the client certificate (e.g. gen-certs.sh esp32-1)
const char *deviceId = "esp32-1";

// Wi-Fi (only used when USE_WIFI = 1 in config.h)
const char wifiSSID[] = "";
const char wifiPass[] = "";

// GPRS / LTE access point of your SIM card operator
const char apn[]      = "internet";
const char gprsUser[] = "";
const char gprsPass[] = "";

// SIM card PIN ("" = no PIN)
#define GSM_PIN ""

// MQTT broker
const char *broker   = "iot-manager.duckdns.org";
const int brokerPort = 8883;

// CA certificate (deploy/certs/ca/ca.crt)
static const char AWS_CERT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
-----END CERTIFICATE-----
)EOF";

// Device certificate (deploy/certs/<deviceId>/client.crt)
static const char AWS_CERT_CRT[] PROGMEM = R"KEY(
-----BEGIN CERTIFICATE-----
-----END CERTIFICATE-----
)KEY";

// Device private key (deploy/certs/<deviceId>/client.key)
static const char AWS_CERT_PRIVATE[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----
-----END RSA PRIVATE KEY-----
)KEY";

#endif // SECRETS_H
