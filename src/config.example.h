#pragma once
// Mau cau hinh. Copy thanh config.h (da bi .gitignore bo qua) roi dien thong tin that.

// ---- Wi-Fi (chi 2.4 GHz) ----
#define WIFI_SSID "CHANGE_ME"
#define WIFI_PASS "CHANGE_ME"
// Wi-Fi du phong (vi du hotspot dien thoai): mat Wi-Fi chinh thi ESP32 tu chuyen sang mang nay
#define WIFI_SSID2 "CHANGE_ME"
#define WIFI_PASS2 "CHANGE_ME"

// ---- MQTT qua TLS (broker EMQX cong cong, dung chung voi firmware lidar) ----
#define MQTT_HOST "broker.emqx.io"
#define MQTT_PORT 8883
#define MQTT_USER ""
#define MQTT_PASS ""

#define DEVICE_ID "DP_SHIP_001"
#define MQTT_TOPIC "bas/" DEVICE_ID "/telemetry"
#define MQTT_CLIENT_ID "bas-" DEVICE_ID "-sim"
#define PUBLISH_INTERVAL_MS 1000UL
#define TZ_OFFSET_S (7 * 3600)
