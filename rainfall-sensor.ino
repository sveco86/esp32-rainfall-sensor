#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WebServer.h>
#include <ElegantOTA.h>
#include <time.h>
#include <esp_timer.h>
#include <math.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

// =======================================================
//  Private configuration
// =======================================================
// Ensure you have a config.h file with:
// SSID_PRIMARY, PASS_PRIMARY, SSID_SECONDARY, PASS_SECONDARY
// MQTT_SERVER, MQTT_USER, MQTT_PASSWORD, MQTT_CLIENT_ID
// IMPULSE_TOPIC, HOURLY_TOPIC, START_TOPIC
// NTP_SERVER_1, NTP_SERVER_2, NTP_SERVER_3, TZ_RULE
// OTA_WEB_USERNAME, OTA_WEB_PASSWORD
#include "config.h"

// =======================================================
//  MQTT buffer config
// =======================================================
#define MQTT_BUFFER_SIZE 4096

// =======================================================
//  Rain gauge setup
// =======================================================
const int   rainfallPin        = 27;       // GPIO27
const float rainfallPerImpulse = 0.28f;    // mm per tip

// Noise / debounce parameters
static const uint32_t MIN_LOW_US     = 5000;    // 5 ms minimum valid low
static const uint32_t REFRACTORY_US  = 20000;   // 20 ms ignore window after valid tip

// =======================================================
//  Connectivity timing configuration
// =======================================================
const unsigned long WIFI_CHECK_INTERVAL_MS = 500;
const unsigned long WIFI_BEGIN_INTERVAL_MS = 15000;
const unsigned long WIFI_RESET_STALE_MS    = 30000;
const unsigned long MQTT_RETRY_INTERVAL_MS = 5000;

// =======================================================
//  State variables
// =======================================================
volatile unsigned long impulseCount        = 0;
volatile bool          impulseDetectedFlag = false;
volatile uint64_t      lowStartUs          = 0;
volatile uint64_t      lastValidTipUs      = 0;

int lastTrackedHour = -1;

// ---- Rolling 7-day × 24-hour rainfall log ----
struct DayHours {
  char  date[11];          // "YYYY-MM-DD"
  float hours[24];
  bool  hasValue[24];
  bool  used;
};

DayHours weekBuf[7];
int currentDayPos = -1;
int dayCount      = 0;

// Wi-Fi + MQTT
WiFiClient espClient;
PubSubClient client(espClient);

// Web OTA
WebServer server(80);

unsigned long lastWifiCheck   = 0;
unsigned long lastWifiBegin   = 0;
unsigned long lastMqttAttempt = 0;
bool wifiConnecting           = false;
int  wifiAttemptCount         = 0;
bool timeInitialized          = false;
bool webOtaInitialized        = false;

// Critical-section lock
portMUX_TYPE rainMux = portMUX_INITIALIZER_UNLOCKED;

// ---- Reusable buffer ----
static char MQTT_OUTBUF[MQTT_BUFFER_SIZE];

// =======================================================
//  Web OTA
// =======================================================
void initWebOTA() {
  server.on("/", HTTP_GET, []() {
    String html;
    html += "<html><head><meta charset='utf-8'><title>ESP32 Rainfall Sensor</title></head><body>";
    html += "<h2>ESP32 Rainfall Sensor</h2>";
    html += "<p>Web OTA is enabled.</p>";
    html += "<p><a href='/update'>Open OTA Update page</a></p>";
    html += "</body></html>";
    server.send(200, "text/html", html);
  });

  ElegantOTA.setAuth(OTA_WEB_USERNAME, OTA_WEB_PASSWORD);
  ElegantOTA.begin(&server);

  ElegantOTA.onStart([]() {
    Serial.println("[OTA-WEB] Start");
  });

  ElegantOTA.onProgress([](size_t current, size_t final) {
    if (final > 0) {
      Serial.printf("[OTA-WEB] Progress: %u%%\n", (unsigned)((current * 100U) / final));
    }
  });

  server.begin();
  webOtaInitialized = true;

  Serial.println("[OTA-WEB] Ready");
  Serial.printf("[OTA-WEB] Open: http://%s/\n", WiFi.localIP().toString().c_str());
  Serial.printf("[OTA-WEB] Update page: http://%s/update\n", WiFi.localIP().toString().c_str());
}

// =======================================================
//  ISR – tipping-bucket pulse-width filter
// =======================================================
void IRAM_ATTR handleRainfall() {
  int level = gpio_get_level((gpio_num_t)rainfallPin);
  uint64_t nowUs = esp_timer_get_time();

  if (nowUs - lastValidTipUs < REFRACTORY_US) return;

  if (level == 0) {
    if (lowStartUs == 0) lowStartUs = nowUs;
  } else {
    if (lowStartUs != 0) {
      uint64_t lowDur = nowUs - lowStartUs;
      lowStartUs = 0;
      if (lowDur >= MIN_LOW_US) {
        portENTER_CRITICAL_ISR(&rainMux);
        impulseCount++;
        impulseDetectedFlag = true;
        portEXIT_CRITICAL_ISR(&rainMux);
        lastValidTipUs = nowUs;
      }
    }
  }
}

// =======================================================
//  Time helpers
// =======================================================
void initTime() {
  Serial.println("[DEBUG] initTime(): calling configTzTime()");
  configTzTime(TZ_RULE, NTP_SERVER_1, NTP_SERVER_2, NTP_SERVER_3);
}

bool nowLocal(struct tm &out) {
  if (!getLocalTime(&out, 1000)) {
    static unsigned long lastErr = 0;
    if (millis() - lastErr > 10000) {
      Serial.println("[DEBUG] getLocalTime() failed");
      lastErr = millis();
    }
    return false;
  }
  return true;
}

void formatDate(const struct tm &t, char out[11]) {
  snprintf(out, 11, "%04d-%02d-%02d", t.tm_year + 1900, t.tm_mon + 1, t.tm_mday);
}

String nowTimeString() {
  struct tm t;
  if (!nowLocal(t)) return String("1970-01-01T00:00:00");
  char buf[25];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d",
           t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
           t.tm_hour, t.tm_min, t.tm_sec);
  return String(buf);
}

// =======================================================
//  7-day rainfall storage
// =======================================================
int findDayIndexByDate(const char date[11]) {
  for (int i = 0; i < 7; ++i)
    if (weekBuf[i].used && strncmp(weekBuf[i].date, date, 11) == 0) return i;
  return -1;
}

int startNewDay(const char date[11]) {
  currentDayPos = (currentDayPos + 1) % 7;
  DayHours &d = weekBuf[currentDayPos];
  strncpy(d.date, date, sizeof(d.date));
  d.date[sizeof(d.date)-1] = '\0';
  for (int h = 0; h < 24; ++h) {
    d.hours[h] = 0.0f;
    d.hasValue[h] = false;
  }
  d.used = true;
  if (dayCount < 7) dayCount++;
  Serial.printf("[DEBUG] startNewDay(): index=%d date=%s dayCount=%d\n",
                currentDayPos, d.date, dayCount);
  return currentDayPos;
}

void setHourValue(const char date[11], int hour, float value) {
  int idx = findDayIndexByDate(date);
  if (idx == -1) idx = startNewDay(date);
  weekBuf[idx].hours[hour] = value;
  weekBuf[idx].hasValue[hour] = true;
  Serial.printf("[DEBUG] setHourValue(): date=%s hour=%d value=%.2f (idx=%d)\n",
                date, hour, value, idx);
}

// Serialize weekly snapshot
static size_t buildWeeklySnapshotJson(char* out, size_t outCap) {
  DynamicJsonDocument doc(8192);

  int order[7];
  int n = 0;
  if (dayCount > 0) {
    int start = (currentDayPos - (dayCount - 1) + 7) % 7;
    for (int i = 0; i < dayCount; ++i) {
      int idx = (start + i) % 7;
      if (weekBuf[idx].used) order[n++] = idx;
    }
  }

  for (int oi = 0; oi < n; ++oi) {
    DayHours &d = weekBuf[order[oi]];
    JsonArray arr = doc.createNestedArray(d.date);
    JsonObject hoursObj = arr.createNestedObject();
    for (int h = 0; h < 24; ++h) {
      if (!d.hasValue[h]) continue;
      char hourKey[9];
      snprintf(hourKey, sizeof(hourKey), "%02d:00:00", h);
      char val[16];
      float v = d.hours[h];
      if (fabs(v - roundf(v)) < 0.005f) snprintf(val, sizeof(val), "%.0f", v);
      else snprintf(val, sizeof(val), "%.2f", v);
      hoursObj[hourKey] = val;
    }
  }

  return serializeJson(doc, out, outCap);
}

// =======================================================
//  Wi-Fi + MQTT connectivity
// =======================================================
void startWifiAttempt(const char *ssid, const char *pwd) {
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);

  Serial.printf("[DEBUG] Starting WiFi connect to SSID: %s\n", ssid);
  WiFi.begin(ssid, pwd);

  wifiConnecting = true;
  lastWifiBegin  = millis();
}

void ensureConnectivity() {
  unsigned long now = millis();
  if (now - lastWifiCheck < WIFI_CHECK_INTERVAL_MS) return;
  lastWifiCheck = now;

  wl_status_t st = WiFi.status();

  if (st == WL_CONNECTED) {
    if (wifiConnecting) {
      Serial.printf("[DEBUG] WiFi connected, IP: %s, RSSI: %d\n",
                    WiFi.localIP().toString().c_str(), WiFi.RSSI());
      wifiConnecting = false;
      wifiAttemptCount = 0;

      if (!timeInitialized) {
        initTime();
        timeInitialized = true;
        Serial.println("[DEBUG] Time initialized");
      }

      if (!webOtaInitialized) {
        initWebOTA();
      }
    }

    if (!client.connected() && (now - lastMqttAttempt) > MQTT_RETRY_INTERVAL_MS) {
      lastMqttAttempt = now;
      client.setServer(MQTT_SERVER, 1883);
      Serial.print("[DEBUG] Connecting to MQTT");

      if (client.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)) {
        Serial.println("\n[DEBUG] MQTT connected");
        {
          String tstr = nowTimeString();
          char payload[96];
          snprintf(payload, sizeof(payload),
                  "{\"status\":\"connected\",\"time\":\"%s\"}",
                  tstr.c_str());
          client.publish(START_TOPIC, (const uint8_t*)payload, strlen(payload), false);
        }

        if (dayCount > 0) {
          size_t n = buildWeeklySnapshotJson(MQTT_OUTBUF, sizeof(MQTT_OUTBUF));
          if (n > 0 && n < sizeof(MQTT_OUTBUF) - 1) {
            bool ok = client.publish(HOURLY_TOPIC, (const uint8_t*)MQTT_OUTBUF, n, true);
            Serial.println(ok ? "[HOURLY] Reconnect snapshot published"
                              : "[HOURLY] Reconnect snapshot publish FAILED");
          }
        }
      } else {
        Serial.printf(" failed, rc=%d\n", client.state());
      }
    }
    return;
  }

  if (wifiConnecting) {
    if ((now - lastWifiBegin) > WIFI_RESET_STALE_MS) {
      Serial.println("[DEBUG] WiFi stuck connecting -> Force Resetting WiFi stack");
      WiFi.disconnect(true);
      delay(100);
      WiFi.mode(WIFI_OFF);
      delay(100);
      wifiConnecting = false;
      webOtaInitialized = false;
    }
    else if ((now - lastWifiBegin) > WIFI_BEGIN_INTERVAL_MS) {
      Serial.println("[DEBUG] Connection timed out (soft). marking as failed.");
      wifiConnecting = false;
      webOtaInitialized = false;
    }
    else {
      return;
    }
  }

  if (!wifiConnecting) {
    const bool usePrimary = (wifiAttemptCount % 2 == 0);
    Serial.printf("\n[DEBUG] WiFi lost/init. Trying %s (Attempt %d)\n",
                  usePrimary ? "Primary" : "Secondary", wifiAttemptCount);

    startWifiAttempt(usePrimary ? SSID_PRIMARY : SSID_SECONDARY,
                     usePrimary ? PASS_PRIMARY : PASS_SECONDARY);
    wifiAttemptCount++;
  }
}

// =======================================================
//  MQTT publish helpers
// =======================================================
void sendImpulseData(float volume, float hourTotal, const String &timeStr) {
  char payload[160];
  snprintf(payload, sizeof(payload),
           "{\"volume\":%.2f,\"hour_total\":%.2f,\"time\":\"%s\"}",
           volume, hourTotal, timeStr.c_str());
  bool ok = client.publish(IMPULSE_TOPIC, (const uint8_t*)payload, strlen(payload), false);

  if (!ok) {
    Serial.println("[DEBUG] Impulse publish failed");
  }
}

void handleHourRollover() {
  struct tm t;
  if (!nowLocal(t)) {
    Serial.println("[DEBUG] nowLocal() failed in handleHourRollover()");
    return;
  }

  int hourNow = t.tm_hour;

  struct tm assignTm = t;
  if (hourNow == 0) {
    time_t ts = mktime(&assignTm);
    ts -= 1;
    localtime_r(&ts, &assignTm);
  }

  char dateStr[11];
  formatDate(assignTm, dateStr);

  Serial.printf("[DEBUG] handleHourRollover(): endHour=%d date=%s\n",
                hourNow, dateStr);

  unsigned long tips;
  portENTER_CRITICAL(&rainMux);
  tips = impulseCount;
  impulseCount = 0;
  portEXIT_CRITICAL(&rainMux);

  float currentRainfallVolume = tips * rainfallPerImpulse;
  Serial.printf("[DEBUG] handleHourRollover(): tips=%lu volume=%.2f\n",
                tips, currentRainfallVolume);

  setHourValue(dateStr, hourNow, currentRainfallVolume);

  if (client.connected()) {
    size_t n = buildWeeklySnapshotJson(MQTT_OUTBUF, sizeof(MQTT_OUTBUF));
    if (n > 0 && n < sizeof(MQTT_OUTBUF) - 1) {
      bool ok = client.publish(HOURLY_TOPIC, (const uint8_t*)MQTT_OUTBUF, n, true);
      Serial.println(ok ? "[HOURLY] Weekly snapshot published"
                        : "[HOURLY] Weekly snapshot publish FAILED");
    } else {
      Serial.printf("[HOURLY] JSON too large for buffer (%u bytes)\n", (unsigned)n);
    }
  } else {
    Serial.println("[HOURLY] MQTT down, snapshot queued.");
  }
}

void maybeSendWeeklySnapshot() {
  struct tm t;
  if (!nowLocal(t)) return;

  int hourNow = t.tm_hour;

  if (lastTrackedHour < 0) {
    lastTrackedHour = hourNow;
    char today[11];
    formatDate(t, today);
    if (findDayIndexByDate(today) == -1) startNewDay(today);
    Serial.printf("[DEBUG] Init lastTrackedHour=%d\n", hourNow);
    return;
  }

  if (hourNow != lastTrackedHour) {
    Serial.printf("[DEBUG] Hour change: %d -> %d\n", lastTrackedHour, hourNow);
    handleHourRollover();
    char today[11];
    formatDate(t, today);
    if (findDayIndexByDate(today) == -1) startNewDay(today);
    lastTrackedHour = hourNow;
  }
}

// =======================================================
//  Setup / Loop
// =======================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.printf("[DEBUG] Boot. MQTT_BUFFER_SIZE=%d\n", MQTT_BUFFER_SIZE);

  for (int i = 0; i < 7; ++i) {
    weekBuf[i].used = false;
    for (int h = 0; h < 24; ++h) {
      weekBuf[i].hours[h] = 0.0f;
      weekBuf[i].hasValue[h] = false;
    }
  }

  pinMode(rainfallPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rainfallPin), handleRainfall, CHANGE);

  client.setServer(MQTT_SERVER, 1883);
  client.setBufferSize(MQTT_BUFFER_SIZE + 512);

  WiFi.mode(WIFI_OFF);
  startWifiAttempt(SSID_PRIMARY, PASS_PRIMARY);
}

void loop() {
  ensureConnectivity();

  if (WiFi.status() == WL_CONNECTED && webOtaInitialized) {
    server.handleClient();
    ElegantOTA.loop();
  }

  if (client.connected()) client.loop();

  bool hadImpulse;
  unsigned long tipsSnapshot;
  portENTER_CRITICAL(&rainMux);
  hadImpulse = impulseDetectedFlag;
  if (hadImpulse) impulseDetectedFlag = false;
  tipsSnapshot = impulseCount;
  portEXIT_CRITICAL(&rainMux);

  if (hadImpulse) {
    String tstr = nowTimeString();
    float currentHourRainfall = tipsSnapshot * rainfallPerImpulse;
    Serial.printf("Rainfall impulse detected at %s | current hour: %.2f mm\n",
                  tstr.c_str(), currentHourRainfall);

    if (client.connected()) {
      sendImpulseData(rainfallPerImpulse, currentHourRainfall, tstr);
    }
  }

  maybeSendWeeklySnapshot();
}
