#pragma once

// ============================================================================
// Credentials and sensible info
// ============================================================================

// // WiFi
#define WIFI_SSID       "wifi_ssid"
#define WIFI_PASSWORD   "wifi_password"

// MQTT
#define MQTT_BROKER       "YourBrokerIp"
#define MQTT_PORT         8883
#define MQTT_USER         "mqtt_user"
#define MQTT_PASSWORD     "mqtt_pass"

// Root certificate for broker MQTT (PEM)
// If left empty, the TLS client will run in insecure mode.
static const char* MQTT_CA_CERT = R"EOF(
)EOF";