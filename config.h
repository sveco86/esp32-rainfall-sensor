#pragma once

// -------------------- Wi-Fi --------------------
#define SSID_PRIMARY       "wifiname1"
#define PASS_PRIMARY       "wifipasswd1"
#define SSID_SECONDARY     "wifiname2"
#define PASS_SECONDARY     "wifipasswd2"

// -------------------- MQTT --------------------
#define MQTT_SERVER        "mqttserverip"
#define MQTT_USER          "mqttuser"
#define MQTT_PASSWORD      "mqttpasswd"
#define MQTT_CLIENT_ID     "ESP32-rfs-1"   // Some unique name

// MQTT Topics
#define IMPULSE_TOPIC      "test/rainfall/impulse"
#define HOURLY_TOPIC       "test/rainfall/hourly"

// -------------------- Time / NTP --------------------
// Timezone rule (Europe/Bratislava): CET / CEST automatic DST
#define TZ_RULE "CET-1CEST,M3.5.0/2,M10.5.0/3"

// Up to 3 NTP servers
#define NTP_SERVER_1 "pool.ntp.org"
#define NTP_SERVER_2 "time.nist.gov"
#define NTP_SERVER_3 "time.google.com"
