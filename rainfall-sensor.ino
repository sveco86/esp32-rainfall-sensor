#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <esp_timer.h>
#include <math.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

// =======================================================
//  Private configuration (not in repo, see config_example.h)
// =======================================================
#include "config.h"

// =======================================================
//  MQTT buffer config (for PubSubClient + our own JSON buffer)
// =======================================================
#define MQTT_BUFFER_SIZE 4096   // bytes, used for PubSubClient buffer & JSON outbuf

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
const unsigned long WIFI_CHECK_INTERVAL_MS = 1000;
const unsigned long WIFI_BEGIN_INTERVAL_MS = 15000;
const unsigned long WIFI_RESET_STALE_MS    = 25000;
const unsigned long MQTT_RETRY_INTERVAL_MS = 3000;

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
  char  date[11];          // "DD.MM.YYYY"
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

// ---- Reusable buffer to avoid heap churn ----
static char MQTT_OUTBUF[MQTT_BUFFER_SIZE]; // for hourly snapshot

// =======================================================
//  ISR – tipping-bucket pulse-width filter
// =======================================================
void IRAM_ATTR handleRainfall() {
  int level = gpio_get_level((gpio_num_t)rainfallPin);
  uint64_t nowUs = esp_timer_get_time();

  if (nowUs - lastValidTipUs < REFRACTORY_US) return;

  if (level == 0) {
    if (lowStartUs == 0) lowStartUs = nowUs;      // start timing LOW
  } else {
    if (lowStartUs != 0) {
      uint64_t lowDur = nowUs - lowStartUs;
      lowStartUs = 0;
      if (lowDur >= MIN_LOW_US) {                 // valid tip
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
    Serial.println("[DEBUG] getLocalTime() failed");
    return false;
  }
  return true;
}

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

// Serialize weekly snapshot directly into provided buffer.
// Returns number of bytes written (0 if not enough space).
static size_t buildWeeklySnapshotJson(char* out, size_t outCap) {
  // 🔍 DEBUG: heap before JSON work
  Serial.printf("[DEBUG] Free heap before JSON: %u bytes\n", ESP.getFreeHeap());

  // Use heap for JSON doc to avoid large stack usage
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
      char hourKey[6]; snprintf(hourKey, sizeof(hourKey), "%d:00", h);
      char val[16];
      float v = d.hours[h];
      if (fabs(v - roundf(v)) < 0.005f) snprintf(val, sizeof(val), "%.0f", v);
      else snprintf(val, sizeof(val), "%.2f", v);
      hoursObj[hourKey] = val;
    }
  }

  size_t written = serializeJson(doc, out, outCap);
  Serial.printf("[DEBUG] buildWeeklySnapshotJson(): written=%u bytes\n", (unsigned)written);

  // 🔍 DEBUG: heap after JSON work (doc will be freed when function returns)
  Serial.printf("[DEBUG] Free heap before JSON: %u bytes\n", ESP.getFreeHeap());

  return written;
}

// =======================================================
//  Wi-Fi + MQTT connectivity
// =======================================================
void startWifiAttempt(const char *ssid, const char *pwd) {
  WiFi.disconnect(false, true);
  delay(50);
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
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
      Serial.printf("[DEBUG] WiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());
      if (!timeInitialized) {
        initTime();
        timeInitialized = true;
        Serial.println("[DEBUG] Time initialized");
      }
    }
    wifiConnecting = false;

    if (!client.connected() && (now - lastMqttAttempt) > MQTT_RETRY_INTERVAL_MS) {
      lastMqttAttempt = now;
      client.setServer(MQTT_SERVER, 1883);
      Serial.print("[DEBUG] Connecting to MQTT");
      if (client.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)) {
        Serial.println("\n[DEBUG] MQTT connected");
        if (dayCount > 0) {
          size_t n = buildWeeklySnapshotJson(MQTT_OUTBUF, sizeof(MQTT_OUTBUF));
          if (n == 0 || n >= sizeof(MQTT_OUTBUF) - 1) {
            Serial.printf("[HOURLY] JSON too large for buffer (%u bytes)\n", (unsigned)n);
          } else {
            bool ok = client.publish(HOURLY_TOPIC,
                                     (const uint8_t*)MQTT_OUTBUF,
                                     n,
                                     true); // retained
            int8_t st = client.state();
            Serial.printf("[HOURLY] published on reconnect (len=%u) result=%s state=%d\n",
                          (unsigned)n, ok ? "OK" : "FAIL", st);
            if (!ok) {
              Serial.println("[DEBUG] Publish on reconnect failed, forcing disconnect");
              client.disconnect();
            }
          }
        }
      } else {
        Serial.print(".");
      }
    }
    return;
  }

  // WiFi not connected
  if (!wifiConnecting) {
    const bool usePrimary = (wifiAttemptCount % 2 == 0);
    Serial.printf("[DEBUG] WiFi not connected, trying %s\n",
                  usePrimary ? "SSID_PRIMARY" : "SSID_SECONDARY");
    startWifiAttempt(usePrimary ? SSID_PRIMARY : SSID_SECONDARY,
                     usePrimary ? PASS_PRIMARY : PASS_SECONDARY);
    wifiAttemptCount++;
    return;
  }

  if ((now - lastWifiBegin) > WIFI_RESET_STALE_MS) {
    Serial.println("[DEBUG] WiFi stuck connecting → resetting STA");
    WiFi.disconnect(false, true);
    delay(100);
    wifiConnecting = false;
  } else if ((now - lastWifiBegin) > WIFI_BEGIN_INTERVAL_MS) {
    Serial.println("[DEBUG] Still connecting...");
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
  int8_t st = client.state();
  Serial.printf("[DEBUG] sendImpulseData(): len=%u result=%s state=%d\n",
                (unsigned)strlen(payload), ok ? "OK" : "FAIL", st);
  if (!ok) {
    Serial.println("[DEBUG] Impulse publish failed, forcing disconnect");
    client.disconnect();
  }
}

void handleHourRollover() {
  struct tm t;
  if (!nowLocal(t)) {
    Serial.println("[DEBUG] nowLocal() failed in handleHourRollover()");
    return;
  }
  int hourNow  = t.tm_hour;
  int prevHour = (hourNow + 23) % 24;

  struct tm prevTm = t;
  if (hourNow == 0) {
    time_t nowEpoch = time(nullptr);
    nowEpoch -= 3600;
    localtime_r(&nowEpoch, &prevTm);
  }

  char dateStr[11];
  formatDate(prevTm, dateStr);

  Serial.printf("[DEBUG] handleHourRollover(): prevHour=%d date=%s\n",
                prevHour, dateStr);

  // atomic read-and-clear of impulseCount
  unsigned long tips;
  portENTER_CRITICAL(&rainMux);
  tips = impulseCount;
  impulseCount = 0;
  portEXIT_CRITICAL(&rainMux);

  float currentRainfallVolume = tips * rainfallPerImpulse;
  Serial.printf("[DEBUG] handleHourRollover(): tips=%lu volume=%.2f\n",
                tips, currentRainfallVolume);

  setHourValue(dateStr, prevHour, currentRainfallVolume);

  if (client.connected()) {
    size_t n = buildWeeklySnapshotJson(MQTT_OUTBUF, sizeof(MQTT_OUTBUF));
    if (n == 0 || n >= sizeof(MQTT_OUTBUF) - 1) {
      Serial.printf("[HOURLY] JSON too large for buffer (%u bytes)\n", (unsigned)n);
    } else {
      bool ok = client.publish(HOURLY_TOPIC,
                               (const uint8_t*)MQTT_OUTBUF,
                               n,
                               true); // retained
      int8_t st = client.state();
      if (ok) {
        Serial.printf("[HOURLY] Weekly snapshot published (len=%u) result=OK state=%d\n",
                      (unsigned)n, st);
      } else {
        Serial.printf("[HOURLY] Weekly snapshot publish FAILED (len=%u) state=%d → forcing disconnect\n",
                      (unsigned)n, st);
        client.disconnect();
      }
    }
  } else {
    Serial.println("[HOURLY] MQTT down, snapshot queued (no publish at rollover).");
  }
}

void maybeSendWeeklySnapshot() {
  struct tm t;
  if (!nowLocal(t)) {
    // already logged inside nowLocal()
    return;
  }
  int hourNow = t.tm_hour;

  if (lastTrackedHour < 0) {
    lastTrackedHour = hourNow;
    char today[11]; formatDate(t, today);
    if (findDayIndexByDate(today) == -1) startNewDay(today);
    Serial.printf("[DEBUG] Init lastTrackedHour=%d, no rollover this hour\n", hourNow);
    return;
  }

  if (hourNow != lastTrackedHour) {
    Serial.printf("[DEBUG] Hour change: lastTrackedHour=%d -> hourNow=%d, calling handleHourRollover()\n",
                  lastTrackedHour, hourNow);
    handleHourRollover();
    char today[11]; formatDate(t, today);
    if (findDayIndexByDate(today) == -1) startNewDay(today);
    lastTrackedHour = hourNow;
  }
}

// =======================================================
//  Setup / Loop
// =======================================================
void setup() {
  Serial.begin(115200);
  delay(100);
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

  startWifiAttempt(SSID_PRIMARY, PASS_PRIMARY);

  client.setServer(MQTT_SERVER, 1883);
  client.setBufferSize(MQTT_BUFFER_SIZE + 512);   // <-- actually resizes PubSubClient buffer and gives 512 extra bytes for Topic string and Packet Headers
  Serial.printf("[DEBUG] PubSubClient buffer set to %d bytes\n", MQTT_BUFFER_SIZE);
}

void loop() {
  ensureConnectivity();
  if (client.connected()) client.loop();

  // snapshot flags and count safely
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
    else
      Serial.println("[DEBUG] MQTT not connected, skipping impulse publish");
  }

  maybeSendWeeklySnapshot();
}
