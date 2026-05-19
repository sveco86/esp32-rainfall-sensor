#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WebServer.h>
#include <ElegantOTA.h>
#include <time.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include <math.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "esp_sntp.h"

// =======================================================
//  Private configuration
// =======================================================
#include "config.h"

// =======================================================
//  MQTT buffer config
// =======================================================
#define MQTT_BUFFER_SIZE 8192

// =======================================================
//  Rain gauge setup
// =======================================================
const int   rainfallPin        = 27;
const float rainfallPerImpulse = 0.28f;

// =======================================================
//  Connectivity timing configuration
// =======================================================
const unsigned long WIFI_CHECK_INTERVAL_MS = 500;
const unsigned long WIFI_BEGIN_INTERVAL_MS = 15000;
const unsigned long WIFI_RESET_STALE_MS    = 30000;
const unsigned long MQTT_RETRY_INTERVAL_MS = 5000;
const unsigned long NTP_RETRY_INTERVAL_MS  = 60000;
const unsigned long TIME_STABLE_MS         = 10000;

// =======================================================
//  Watchdog configuration
// =======================================================
static const uint32_t WDT_TIMEOUT_MS = 10000;

// =======================================================
//  State variables
// =======================================================
volatile unsigned long impulseCount        = 0;
volatile bool          impulseDetectedFlag = false;
volatile uint64_t      lastValidTipUs      = 0;
volatile unsigned long lastSentImpulseCount = 0;

int  lastTrackedHour    = -1;
bool wdtRegistered      = false;
bool rainfallClockReady = false;
int mqttFailCount = 0;

// ---- Rolling 7-day × 24-hour rainfall log ----
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

// Web OTA
WebServer server(80);

unsigned long lastWifiCheck   = 0;
unsigned long lastWifiBegin   = 0;
unsigned long lastMqttAttempt = 0;
bool wifiConnecting           = false;
int  wifiAttemptCount         = 0;
bool ntpRequestStarted        = false;
bool timeValid                = false;
unsigned long lastNtpAttempt  = 0;
bool webOtaInitialized        = false;

// --- Time stability tracking ---
time_t lastAcceptedTime         = 0;
unsigned long timeStableSinceMs = 0;
bool ntpTimeStable              = false;
bool startTopicSentForSession   = false;
volatile bool sntpSynced        = false;

// Critical-section lock
portMUX_TYPE rainMux = portMUX_INITIALIZER_UNLOCKED;

// ---- Reusable buffer ----
static char MQTT_OUTBUF[MQTT_BUFFER_SIZE];

// =======================================================
//  Helpers
// =======================================================
const char* resetReasonToStr(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT";
    case ESP_RST_SW:        return "SW";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    default:                return "UNKNOWN";
  }
}

bool isTimeReasonable() {
  time_t now = time(nullptr);
  if (now <= 0) return false;

  const time_t MIN_REASONABLE = 1735689600; // 2025-01-01 00:00:00 UTC
  const time_t MAX_REASONABLE = 2051222400; // 2035-01-01 00:00:00 UTC

  return (now >= MIN_REASONABLE && now <= MAX_REASONABLE);
}

void logCurrentTimeDebug(const char* prefix) {
  time_t now = time(nullptr);
  if (now <= 0) {
    Serial.printf("[DEBUG] %s | time(nullptr) invalid: %lld\n", prefix, (long long)now);
    return;
  }

  struct tm lt;
  struct tm ut;
  char lbuf[32];
  char ubuf[32];

  localtime_r(&now, &lt);
  gmtime_r(&now, &ut);

  strftime(lbuf, sizeof(lbuf), "%Y-%m-%d %H:%M:%S", &lt);
  strftime(ubuf, sizeof(ubuf), "%Y-%m-%d %H:%M:%S", &ut);

  Serial.printf("[DEBUG] %s | local=%s | utc=%s | epoch=%lld\n",
                prefix, lbuf, ubuf, (long long)now);
}

void registerWatchdogForCurrentTask() {
  if (!wdtRegistered) {
    esp_task_wdt_add(NULL);
    wdtRegistered = true;
    Serial.println("[WDT] Registered current task");
  }
}

void unregisterWatchdogForCurrentTask() {
  if (wdtRegistered) {
    esp_task_wdt_delete(NULL);
    wdtRegistered = false;
    Serial.println("[WDT] Unregistered current task");
  }
}

void applyTimezone() {
  setenv("TZ", TZ_RULE, 1);
  tzset();
  Serial.printf("[DEBUG] Timezone applied: %s\n", TZ_RULE);
}

void onTimeSync(struct timeval *tv) {
  sntpSynced = true;
  Serial.println("[NTP] Time sync callback received");
  logCurrentTimeDebug("SNTP callback");
}

void clearWeeklyBuffer() {
  for (int i = 0; i < 7; ++i) {
    weekBuf[i].used = false;
    weekBuf[i].date[0] = '\0';
    for (int h = 0; h < 24; ++h) {
      weekBuf[i].hours[h] = 0.0f;
      weekBuf[i].hasValue[h] = false;
    }
  }
  currentDayPos = -1;
  dayCount = 0;
  Serial.println("[DEBUG] Weekly rainfall buffer cleared");
}

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
    unregisterWatchdogForCurrentTask();
    Serial.println("[OTA-WEB] Start");
  });

  ElegantOTA.onProgress([](size_t current, size_t final) {
    if (final > 0) {
      Serial.printf("[OTA-WEB] Progress: %u%%\n", (unsigned)((current * 100U) / final));
    }
  });

  ElegantOTA.onEnd([](bool success) {
    registerWatchdogForCurrentTask();
    Serial.printf("[OTA-WEB] End. Success=%s\n", success ? "true" : "false");
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
  uint64_t nowUs = esp_timer_get_time();
  
  // Ak od posledného zopnutia prešlo viac ako 50 ms (50000 mikrosekúnd)
  if (nowUs - lastValidTipUs > 50000) { 
    portENTER_CRITICAL_ISR(&rainMux);
    impulseCount++;
    impulseDetectedFlag = true;
    portEXIT_CRITICAL_ISR(&rainMux);
    
    lastValidTipUs = nowUs; // Zaznamenaj čas tohto preklopenia
  }
}

// =======================================================
//  Time helpers
// =======================================================
void resetTimeSyncState() {
  timeValid = false;
  ntpTimeStable = false;
  rainfallClockReady = false;
  ntpRequestStarted = false;
  lastAcceptedTime = 0;
  timeStableSinceMs = 0;
  startTopicSentForSession = false;
  sntpSynced = false;
}

void startNtpSync() {
  Serial.println("[DEBUG] startNtpSync(): calling configTzTime()");
  sntpSynced = false;
  sntp_set_sync_status(SNTP_SYNC_STATUS_RESET);
  sntp_set_time_sync_notification_cb(onTimeSync);
  configTzTime(TZ_RULE, NTP_SERVER_1, NTP_SERVER_2, NTP_SERVER_3);
  ntpRequestStarted = true;
  lastNtpAttempt = millis();
}

void maintainTimeSync() {
  if (WiFi.status() != WL_CONNECTED) {
    resetTimeSyncState();
    return;
  }

  const unsigned long nowMs = millis();

  if (!ntpRequestStarted) {
    Serial.println("[DEBUG] NTP not started yet");
    startNtpSync();
    return;
  }

  sntp_sync_status_t syncStatus = sntp_get_sync_status();

  if (!sntpSynced && syncStatus != SNTP_SYNC_STATUS_COMPLETED) {
    timeValid = false;
    ntpTimeStable = false;
    rainfallClockReady = false;
    // ODOBRANÉ cyklické volanie startNtpSync(). lwIP sa pokúša o reconnect sám na pozadí.
    return;
  }

  if (!isTimeReasonable()) {
    timeValid = false;
    ntpTimeStable = false;
    rainfallClockReady = false;

    if ((nowMs - lastNtpAttempt) >= NTP_RETRY_INTERVAL_MS) {
      Serial.println("[DEBUG] Time synced but unreasonable, retrying NTP");
      startNtpSync();
    }
    return;
  }

  time_t now = time(nullptr);

  if (lastAcceptedTime == 0) {
    lastAcceptedTime = now;
    timeStableSinceMs = nowMs;
    timeValid = false;
    ntpTimeStable = false;
    logCurrentTimeDebug("First SNTP-synced reasonable time seen");
    return;
  }

  if (labs((long)(now - lastAcceptedTime)) > 5) {
    Serial.printf("[DEBUG] Time jump detected after SNTP: prev=%lld now=%lld diff=%ld s\n",
                  (long long)lastAcceptedTime, (long long)now, (long)(now - lastAcceptedTime));
    lastAcceptedTime = now;
    timeStableSinceMs = nowMs;
    timeValid = false;
    ntpTimeStable = false;
    logCurrentTimeDebug("Time jump reset");
    return;
  }

  lastAcceptedTime = now;

  if ((nowMs - timeStableSinceMs) >= TIME_STABLE_MS) {
    if (!ntpTimeStable) {
      ntpTimeStable = true;
      timeValid = true;
      logCurrentTimeDebug("Time became stable after confirmed SNTP sync");
    } else {
      timeValid = true;
    }
    return;
  }

  timeValid = false;
  ntpTimeStable = false;
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
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &t);
  return String(buf);
}

// =======================================================
//  7-day rainfall storage
// =======================================================
int findDayIndexByDate(const char date[11]) {
  for (int i = 0; i < 7; ++i) {
    if (weekBuf[i].used && strncmp(weekBuf[i].date, date, 11) == 0) return i;
  }
  return -1;
}

int startNewDay(const char date[11]) {
  currentDayPos = (currentDayPos + 1) % 7;
  DayHours &d = weekBuf[currentDayPos];
  strncpy(d.date, date, sizeof(d.date));
  d.date[sizeof(d.date) - 1] = '\0';

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
  if (hour < 0 || hour > 23) {
    Serial.printf("[ERROR] setHourValue(): invalid hour=%d for date=%s\n", hour, date);
    return;
  }

  int idx = findDayIndexByDate(date);
  if (idx == -1) idx = startNewDay(date);

  weekBuf[idx].hours[hour] = value;
  weekBuf[idx].hasValue[hour] = true;

  Serial.printf("[DEBUG] setHourValue(): date=%s hour=%d value=%.2f (idx=%d)\n",
                date, hour, value, idx);
}

// =======================================================
//  Serialize weekly snapshot
// =======================================================
static size_t buildWeeklySnapshotJson(char* out, size_t outCap) {
  JsonDocument doc;

  int order[7];
  int n = 0;

  if (dayCount > 0) {
    int start = (currentDayPos - (dayCount - 1) + 7) % 7;
    for (int i = 0; i < dayCount; ++i) {
      int idx = (start + i) % 7;
      if (weekBuf[idx].used) order[n++] = idx;
    }
  }

  int daysAdded = 0;

  for (int oi = 0; oi < n; ++oi) {
    DayHours &d = weekBuf[order[oi]];

    bool hasAnyHour = false;
    for (int h = 0; h < 24; ++h) {
      if (d.hasValue[h]) {
        hasAnyHour = true;
        break;
      }
    }

    if (!hasAnyHour) continue;

    JsonArray arr = doc[d.date].to<JsonArray>();
    JsonObject hoursObj = arr.add<JsonObject>();

    for (int h = 0; h < 24; ++h) {
      if (!d.hasValue[h]) continue;

      char hourKey[9];
      snprintf(hourKey, sizeof(hourKey), "%02d:59:59", h);

      char val[16];
      float v = d.hours[h];
      if (fabs(v - roundf(v)) < 0.005f) {
        snprintf(val, sizeof(val), "%.0f", v);
      } else {
        snprintf(val, sizeof(val), "%.2f", v);
      }

      hoursObj[hourKey] = val;
    }

    daysAdded++;
  }

  if (daysAdded == 0) {
    Serial.println("[DEBUG] No hourly values yet, skipping empty snapshot");
    return 0;
  }

  if (doc.overflowed()) {
    Serial.println("[ERROR] JSON document overflowed!");
    Serial.printf("[DEBUG] Free heap at overflow: %u bytes\n", ESP.getFreeHeap());
    return 0;
  }

  size_t jsonLen = measureJson(doc);
  Serial.printf("[DEBUG] JSON measured size: %u bytes (buffer: %u)\n",
                (unsigned)jsonLen, (unsigned)outCap);

  if (jsonLen >= outCap) {
    Serial.println("[ERROR] JSON does not fit into MQTT buffer!");
    return 0;
  }

  size_t written = serializeJson(doc, out, outCap);
  if (written == 0) {
    Serial.println("[ERROR] serializeJson() failed");
    return 0;
  }

  return written;
}

// =======================================================
//  MQTT publish helpers
// =======================================================
void publishStartTopicIfReady() {
  if (!client.connected()) return;
  if (startTopicSentForSession) return;
  if (!timeValid || !ntpTimeStable) return;

  String tstr = nowTimeString();
  char payload[96];
  snprintf(payload, sizeof(payload),
           "{\"status\":\"connected\",\"time\":\"%s\"}",
           tstr.c_str());

  bool ok = client.publish(START_TOPIC, (const uint8_t*)payload, strlen(payload), false);
  if (ok) {
    startTopicSentForSession = true;
    Serial.printf("[DEBUG] START_TOPIC published with stable time: %s\n", tstr.c_str());
  } else {
    Serial.println("[DEBUG] START_TOPIC publish failed");
  }
}

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
  struct tm nowTm;
  if (!nowLocal(nowTm)) {
    Serial.println("[DEBUG] nowLocal() failed in handleHourRollover()");
    return;
  }

  time_t nowTs = mktime(&nowTm);
  if (nowTs <= 0) {
    Serial.println("[DEBUG] mktime() failed in handleHourRollover()");
    return;
  }

  time_t bucketEndTs = nowTs - 1;
  struct tm bucketEndTm;
  localtime_r(&bucketEndTs, &bucketEndTm);

  char dateStr[11];
  formatDate(bucketEndTm, dateStr);
  int bucketHour = bucketEndTm.tm_hour;

  char dbg[32];
  strftime(dbg, sizeof(dbg), "%Y-%m-%d %H:%M:%S", &bucketEndTm);
  Serial.printf("[DEBUG] handleHourRollover(): bucket end timestamp = %s\n", dbg);

  unsigned long tips;
  portENTER_CRITICAL(&rainMux);
  tips = impulseCount;
  impulseCount = 0;
  lastSentImpulseCount = 0;
  portEXIT_CRITICAL(&rainMux);

  float currentRainfallVolume = tips * rainfallPerImpulse;
  Serial.printf("[DEBUG] handleHourRollover(): tips=%lu volume=%.2f\n",
                tips, currentRainfallVolume);

  setHourValue(dateStr, bucketHour, currentRainfallVolume);

  if (client.connected()) {
    size_t n = buildWeeklySnapshotJson(MQTT_OUTBUF, sizeof(MQTT_OUTBUF));
    if (n > 0) {
      bool ok = client.publish(HOURLY_TOPIC, (const uint8_t*)MQTT_OUTBUF, n, true);
      Serial.println(ok ? "[HOURLY] Weekly snapshot published"
                        : "[HOURLY] Weekly snapshot publish FAILED");
    } else {
      Serial.println("[HOURLY] No non-empty snapshot to publish");
    }
  } else {
    Serial.println("[HOURLY] MQTT down, snapshot queued.");
  }
}

// =======================================================
//  Wi-Fi + MQTT connectivity
// =======================================================
void startWifiAttempt(const char *ssid, const char *pwd) {
  WiFi.disconnect(false);
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

      if (!webOtaInitialized) {
        initWebOTA();
      }
    }

    maintainTimeSync();

  if (!client.connected() && (now - lastMqttAttempt) > MQTT_RETRY_INTERVAL_MS) {
      lastMqttAttempt = now;
      client.setServer(MQTT_SERVER, 1883);
      Serial.print("[DEBUG] Connecting to MQTT");

      if (client.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)) {
        Serial.println("\n[DEBUG] MQTT connected");
        startTopicSentForSession = false;
        mqttFailCount = 0; // Reset počítadla pri úspechu

        // Hourly snapshot sa po reconnecte NESMIE automaticky posielať.
        Serial.println("[HOURLY] Reconnect snapshot suppressed");
      } else {
        Serial.printf(" failed, rc=%d\n", client.state());
        mqttFailCount++;

        // Ak zlyhá pripojenie 12x za sebou (cca 1 minúta permanentného výpadku)
        if (mqttFailCount >= 12) {
          Serial.println("[DEBUG] MQTT failed repeatedly. Forcing hard Wi-Fi reset to clear sockets...");
          WiFi.disconnect(false);
          delay(100);
          WiFi.mode(WIFI_OFF);
          delay(100);
          
          wifiConnecting = false;
          webOtaInitialized = false;
          resetTimeSyncState(); // Toto vráti ntpRequestStarted na false, takže po nábehu Wi-Fi znova inicializuje čistý SNTP dopyt
          mqttFailCount = 0;
          return; 
        }
      }
    }

    publishStartTopicIfReady();
    return;
  }

  if (wifiConnecting) {
    if ((now - lastWifiBegin) > WIFI_RESET_STALE_MS) {
      Serial.println("[DEBUG] WiFi stuck connecting -> Force Resetting WiFi stack");
      WiFi.disconnect(false);
      delay(100);
      WiFi.mode(WIFI_OFF);
      delay(100);
      wifiConnecting = false;
      webOtaInitialized = false;
      resetTimeSyncState();
    } else if ((now - lastWifiBegin) > WIFI_BEGIN_INTERVAL_MS) {
      Serial.println("[DEBUG] Connection timed out (soft). marking as failed.");
      wifiConnecting = false;
      webOtaInitialized = false;
      resetTimeSyncState();
    } else {
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

void maybeSendWeeklySnapshot() {
  struct tm t;
  if (!nowLocal(t) || !timeValid || !ntpTimeStable) {
    static unsigned long lastWarn = 0;
    if (millis() - lastWarn > 30000) {
      Serial.println("[DEBUG] maybeSendWeeklySnapshot(): time not stable yet");
      lastWarn = millis();
    }
    return;
  }

  int hourNow = t.tm_hour;

  if (!rainfallClockReady) {
    // Pri prvom stabilnom čase zahodíme všetko, čo mohlo vzniknúť pred SNTP sync
    clearWeeklyBuffer();

    // OPRAVA: Vynuluj počítadlá impulzov nazbierané počas bootu
    portENTER_CRITICAL(&rainMux);
    impulseCount = 0;
    lastSentImpulseCount = 0;
    impulseDetectedFlag = false;
    portEXIT_CRITICAL(&rainMux);

    lastTrackedHour = hourNow;

    char today[11];
    formatDate(t, today);
    startNewDay(today);
    
    rainfallClockReady = true;
    Serial.printf("[DEBUG] Rainfall clock READY at %02d:00. Pre-sync data cleared.\n", hourNow);
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
  Serial.printf("[BOOT] Reset reason: %s\n", resetReasonToStr(esp_reset_reason()));
  Serial.printf("[DEBUG] Free heap at boot: %u bytes\n", ESP.getFreeHeap());

  applyTimezone();

  time_t bootNow = time(nullptr);
  Serial.printf("[DEBUG] Raw time at boot before WiFi/NTP: %lld\n", (long long)bootNow);
  logCurrentTimeDebug("Boot time before sync");

  esp_task_wdt_config_t twdt_config = {
    .timeout_ms = WDT_TIMEOUT_MS,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_task_wdt_init(&twdt_config);
  registerWatchdogForCurrentTask();

  clearWeeklyBuffer();

  pinMode(rainfallPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rainfallPin), handleRainfall, FALLING);


  client.setServer(MQTT_SERVER, 1883);
  client.setBufferSize(MQTT_BUFFER_SIZE + 512);

  Serial.printf("[DEBUG] MQTT client buffer size set to: %u bytes\n",
                (unsigned)(MQTT_BUFFER_SIZE + 512));

  resetTimeSyncState();

  WiFi.mode(WIFI_OFF);
  delay(100);
  startWifiAttempt(SSID_PRIMARY, PASS_PRIMARY);
}

void loop() {
  if (wdtRegistered) esp_task_wdt_reset();

  ensureConnectivity();

  if (wdtRegistered) esp_task_wdt_reset();

  if (WiFi.status() == WL_CONNECTED && webOtaInitialized) {
    server.handleClient();
    ElegantOTA.loop();
  }

  if (wdtRegistered) esp_task_wdt_reset();

  if (client.connected()) client.loop();

  bool hadImpulse;
  unsigned long tipsSnapshot;
  portENTER_CRITICAL(&rainMux);
  hadImpulse = impulseDetectedFlag;
  if (hadImpulse) impulseDetectedFlag = false;
  tipsSnapshot = impulseCount;
  portEXIT_CRITICAL(&rainMux);

  if (hadImpulse) {
    if (rainfallClockReady && timeValid && ntpTimeStable) {
      String tstr = nowTimeString();
      
      // Vypočítame koľko impulzov reálne prebehlo od POSLEDNÉHO ÚSPEŠNÉHO odoslania
      unsigned long deltaTips = tipsSnapshot - lastSentImpulseCount;
      float deltaVolume = deltaTips * rainfallPerImpulse;
      float currentHourRainfall = tipsSnapshot * rainfallPerImpulse;

      if (deltaTips > 0) {
        Serial.printf("Rainfall impulse detected at %s | delta: %.2f mm | hour total: %.2f mm\n",
                      tstr.c_str(), deltaVolume, currentHourRainfall);

        if (client.connected()) {
          sendImpulseData(deltaVolume, currentHourRainfall, tstr); 
          // OPRAVA: Počítadlo posunúť SEM. Aktualizuje sa len pri online stave.
          lastSentImpulseCount = tipsSnapshot; 
        } else {
          Serial.println("[MQTT] Disconnected. Delta accumulation in progress...");
        }
      }
    } else {
      Serial.println("[DEBUG] Impulse ignored (waiting for stable clock sync)");
    }
  }

  maybeSendWeeklySnapshot();

  if (wdtRegistered) esp_task_wdt_reset();
}
