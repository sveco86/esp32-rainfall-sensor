#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <esp_timer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

// =======================================================
//  Private configuration (not in repo, see config_example.h)
// =======================================================
#include "config.h"

// =======================================================
//  Rain gauge setup
// =======================================================
const int   rainfallPin        = 27;       // GPIO27
const float rainfallPerImpulse = 0.28f;    // mm per tip

// Noise / debounce parameters
static const uint32_t MIN_LOW_US     = 5000;   // 5 ms minimum valid low
static const uint32_t REFRACTORY_US  = 20000;  // 20 ms ignore window after valid tip

// =======================================================
//  Connectivity timing configuration
// =======================================================
const unsigned long WIFI_CHECK_INTERVAL_MS = 1000;
const unsigned long WIFI_BEGIN_INTERVAL_MS = 15000;
const unsigned long WIFI_RESET_STALE_MS    = 25000;
const unsigned long MQTT_RETRY_INTERVAL_MS = 3000;

// =======================================================
//  State variables
// =======================================================
volatile unsigned long impulseCount = 0;
volatile bool impulseDetectedFlag   = false;
volatile uint64_t lowStartUs        = 0;
volatile uint64_t lastValidTipUs    = 0;

int lastTrackedHour = -1;
int lastTrackedWday = -1;

struct DayHours {
  char  date[11];
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

unsigned long lastWifiCheck   = 0;
unsigned long lastWifiBegin   = 0;
unsigned long lastMqttAttempt = 0;
bool wifiConnecting           = false;
int  wifiAttemptCount         = 0;
bool timeInitialized          = false;

// Critical-section lock
portMUX_TYPE rainMux = portMUX_INITIALIZER_UNLOCKED;

// =======================================================
//  ISR – tipping-bucket pulse-width filter
// =======================================================
void IRAM_ATTR handleRainfall() {
  int level = gpio_get_level((gpio_num_t)rainfallPin);
  uint64_t nowUs = esp_timer_get_time();

  if (nowUs - lastValidTipUs < REFRACTORY_US) return;

  if (level == 0) {
    if (lowStartUs == 0) lowStartUs = nowUs;          // start timing
  } else {
    if (lowStartUs != 0) {
      uint64_t lowDur = nowUs - lowStartUs;
      lowStartUs = 0;
      if (lowDur >= MIN_LOW_US) {                     // valid tip
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
void initTime() { configTzTime(TZ_RULE, NTP_SERVER_1, NTP_SERVER_2, NTP_SERVER_3); }

bool nowLocal(struct tm &out) { return getLocalTime(&out, 1000); }

void formatDate(const struct tm &t, char out[11]) {
  snprintf(out, 11, "%02d.%02d.%04d", t.tm_mday, t.tm_mon + 1, t.tm_year + 1900);
}

String nowTimeString() {
  struct tm t;
  if (!nowLocal(t)) return String("00:00:00");
  char buf[16];
  snprintf(buf, sizeof(buf), "%02d:%02d:%02d", t.tm_hour, t.tm_min, t.tm_sec);
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
  strncpy(d.date, date, 11);
  for (int h = 0; h < 24; ++h) { d.hours[h] = 0.0f; d.hasValue[h] = false; }
  d.used = true;
  if (dayCount < 7) dayCount++;
  return currentDayPos;
}

void setHourValue(const char date[11], int hour, float value) {
  int idx = findDayIndexByDate(date);
  if (idx == -1) idx = startNewDay(date);
  weekBuf[idx].hours[hour] = value;
  weekBuf[idx].hasValue[hour] = true;
}

String buildWeeklySnapshotJson() {
  DynamicJsonDocument doc(8192);
  int order[7]; int n = 0;
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
      char hourKey[6]; snprintf(hourKey, sizeof(hourKey), "%d:00", h);
      char val[16];
      float v = d.hours[h];
      if (fabs(v - roundf(v)) < 0.005f) snprintf(val, sizeof(val), "%.0f", v);
      else snprintf(val, sizeof(val), "%.2f", v);
      hoursObj[hourKey] = val;
    }
  }
  String out; serializeJson(doc, out);
  return out;
}

// =======================================================
//  Wi-Fi + MQTT connectivity
// =======================================================
void startWifiAttempt(const char *ssid, const char *pwd) {
  WiFi.disconnect(false, true); delay(50);
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA); WiFi.setAutoReconnect(false);
  Serial.printf("Starting WiFi connect to SSID: %s\n", ssid);
  WiFi.begin(ssid, pwd);
  wifiConnecting = true; lastWifiBegin = millis();
}

void ensureConnectivity() {
  unsigned long now = millis();
  if (now - lastWifiCheck < WIFI_CHECK_INTERVAL_MS) return;
  lastWifiCheck = now;

  if (WiFi.status() == WL_CONNECTED) {
    if (wifiConnecting) {
      Serial.printf("WiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());
      if (!timeInitialized) { initTime(); timeInitialized = true; }
    }
    wifiConnecting = false;

    if (!client.connected() && (now - lastMqttAttempt) > MQTT_RETRY_INTERVAL_MS) {
      lastMqttAttempt = now; client.setServer(MQTT_SERVER, 1883);
      Serial.print("Connecting to MQTT");
      if (client.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)) {
        Serial.println("\nMQTT connected");
        if (dayCount > 0) {
          String snapshot = buildWeeklySnapshotJson();
          client.publish(HOURLY_TOPIC,
                         (const uint8_t*)snapshot.c_str(),
                         snapshot.length(),
                         true); // retained
        }
      } else Serial.print(".");
    }
    return;
  }

  if (!wifiConnecting) {
    const bool usePrimary = (wifiAttemptCount % 2 == 0);
    startWifiAttempt(usePrimary ? SSID_PRIMARY : SSID_SECONDARY,
                     usePrimary ? PASS_PRIMARY : PASS_SECONDARY);
    wifiAttemptCount++;
    return;
  }

  if ((now - lastWifiBegin) > WIFI_RESET_STALE_MS) {
    Serial.println("WiFi stuck connecting → resetting STA");
    WiFi.disconnect(false, true); delay(100);
    wifiConnecting = false;
  } else if ((now - lastWifiBegin) > WIFI_BEGIN_INTERVAL_MS) {
    Serial.println("Still connecting...");
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
  client.publish(IMPULSE_TOPIC, (const uint8_t*)payload, strlen(payload), false);
}

void handleHourRollover() {
  struct tm t; if (!nowLocal(t)) return;
  int hourNow = t.tm_hour;
  int prevHour = (hourNow + 23) % 24;
  struct tm prevTm = t;
  if (hourNow == 0) {
    time_t nowEpoch = time(nullptr); nowEpoch -= 3600; localtime_r(&nowEpoch, &prevTm);
  }

  char dateStr[11]; formatDate(prevTm, dateStr);

  unsigned long tips;
  portENTER_CRITICAL(&rainMux);
  tips = impulseCount; impulseCount = 0;
  portEXIT_CRITICAL(&rainMux);

  float currentRainfallVolume = tips * rainfallPerImpulse;
  setHourValue(dateStr, prevHour, currentRainfallVolume);

  if (client.connected()) {
    String snapshot = buildWeeklySnapshotJson();
    bool ok = client.publish(HOURLY_TOPIC,
                             (const uint8_t*)snapshot.c_str(),
                             snapshot.length(),
                             true);
    Serial.println(ok ? "[HOURLY] Weekly snapshot published (retained)" :
                        "[HOURLY] Publish FAILED");
  } else Serial.println("[HOURLY] MQTT down, snapshot queued.");
}

void maybeSendWeeklySnapshot() {
  struct tm t; if (!nowLocal(t)) return;
  int hourNow = t.tm_hour; int wdayNow = t.tm_wday;

  if (lastTrackedHour < 0) {
    lastTrackedHour = hourNow; lastTrackedWday = wdayNow;
    char today[11]; formatDate(t, today);
    if (findDayIndexByDate(today) == -1) startNewDay(today);
    return;
  }
  if (hourNow != lastTrackedHour) {
    handleHourRollover();
    char today[11]; formatDate(t, today);
    if (findDayIndexByDate(today) == -1) startNewDay(today);
    lastTrackedHour = hourNow; lastTrackedWday = wdayNow;
  }
}

// =======================================================
//  Setup / Loop
// =======================================================
void setup() {
  Serial.begin(115200); delay(100);

  for (int i = 0; i < 7; ++i) {
    weekBuf[i].used = false;
    for (int h = 0; h < 24; ++h) {
      weekBuf[i].hours[h] = 0.0f; weekBuf[i].hasValue[h] = false;
    }
  }

  pinMode(rainfallPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rainfallPin), handleRainfall, CHANGE);

  startWifiAttempt(SSID_PRIMARY, PASS_PRIMARY);
  client.setServer(MQTT_SERVER, 1883);
}

void loop() {
  ensureConnectivity();
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
    Serial.printf("Rainfall impulse detected at %s | current hour rainfall: %.2f mm\n",
                  tstr.c_str(), currentHourRainfall);
    if (client.connected())
      sendImpulseData(rainfallPerImpulse, currentHourRainfall, tstr);
  }

  maybeSendWeeklySnapshot();
}
