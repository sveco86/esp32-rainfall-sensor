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
//  Private configuration
// =======================================================
// Ensure you have a config.h file with:
// SSID_PRIMARY, PASS_PRIMARY, SSID_SECONDARY, PASS_SECONDARY
// MQTT_SERVER, MQTT_USER, MQTT_PASSWORD, MQTT_CLIENT_ID
// IMPULSE_TOPIC, HOURLY_TOPIC
// NTP_SERVER_1, NTP_SERVER_2, NTP_SERVER_3, TZ_RULE
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
const unsigned long WIFI_CHECK_INTERVAL_MS = 500;   // Check status more frequently
const unsigned long WIFI_BEGIN_INTERVAL_MS = 15000; // Time to wait for connection
const unsigned long WIFI_RESET_STALE_MS    = 30000; // Force radio reset if stuck here
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

// ---- Reusable buffer ----
static char MQTT_OUTBUF[MQTT_BUFFER_SIZE]; 

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
    // Only print error occasionally to avoid serial spam
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

// Serialize weekly snapshot
static size_t buildWeeklySnapshotJson(char* out, size_t outCap) {
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
  return serializeJson(doc, out, outCap);
}

// =======================================================
//  Wi-Fi + MQTT connectivity (FIXED)
// =======================================================
void startWifiAttempt(const char *ssid, const char *pwd) {
  // FIX 1: Don't use 'true' in disconnect (avoids flash erase)
  WiFi.disconnect(); 
  
  // FIX 2: Set mode explicitly
  WiFi.mode(WIFI_STA);
  
  // FIX 3: Disable WiFi power saving to prevent router disconnects
  WiFi.setSleep(false); 

  WiFi.persistent(false); // Don't save new credentials to flash
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

  // --- Case 1: Fully Connected ---
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
    }

    // Handle MQTT
    if (!client.connected() && (now - lastMqttAttempt) > MQTT_RETRY_INTERVAL_MS) {
      lastMqttAttempt = now;
      client.setServer(MQTT_SERVER, 1883);
      Serial.print("[DEBUG] Connecting to MQTT");
      
      if (client.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)) {
        Serial.println("\n[DEBUG] MQTT connected");
        
        // Publish retained snapshot on reconnect
        if (dayCount > 0) {
          size_t n = buildWeeklySnapshotJson(MQTT_OUTBUF, sizeof(MQTT_OUTBUF));
          if (n > 0 && n < sizeof(MQTT_OUTBUF) - 1) {
            client.publish(HOURLY_TOPIC, (const uint8_t*)MQTT_OUTBUF, n, true);
          }
        }
      } else {
        Serial.printf(" failed, rc=%d\n", client.state());
      }
    }
    return;
  }

  // --- Case 2: Currently connecting (wait phase) ---
  if (wifiConnecting) {
    // If it has been too long (Stuck connecting state)
    if ((now - lastWifiBegin) > WIFI_RESET_STALE_MS) {
      Serial.println("[DEBUG] WiFi stuck connecting -> Force Resetting WiFi stack");
      
      // Hard reset of the WiFi radio
      WiFi.disconnect(true); // erase once to clear stack
      delay(100);
      WiFi.mode(WIFI_OFF);   // Turn radio off completely
      delay(100);
      
      wifiConnecting = false; // Allow the next block to restart the process
    } 
    else if ((now - lastWifiBegin) > WIFI_BEGIN_INTERVAL_MS) {
       Serial.println("[DEBUG] Connection timed out (soft). marking as failed.");
       wifiConnecting = false;
    }
    else {
      // Still waiting, do nothing (or print dot)
      return;
    }
  }

  // --- Case 3: Disconnected and not currently trying (Start new attempt) ---
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
    // Don't force disconnect here, let ensureConnectivity handle it
  }
}

void handleHourRollover() {
  struct tm t;
  if (!nowLocal(t)) return;

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

  // atomic read-and-clear
  unsigned long tips;
  portENTER_CRITICAL(&rainMux);
  tips = impulseCount;
  impulseCount = 0;
  portEXIT_CRITICAL(&rainMux);

  float currentRainfallVolume = tips * rainfallPerImpulse;
  setHourValue(dateStr, prevHour, currentRainfallVolume);

  if (client.connected()) {
    size_t n = buildWeeklySnapshotJson(MQTT_OUTBUF, sizeof(MQTT_OUTBUF));
    if (n > 0 && n < sizeof(MQTT_OUTBUF) - 1) {
      client.publish(HOURLY_TOPIC, (const uint8_t*)MQTT_OUTBUF, n, true); // retained
      Serial.println("[HOURLY] Weekly snapshot published");
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
    char today[11]; formatDate(t, today);
    if (findDayIndexByDate(today) == -1) startNewDay(today);
    Serial.printf("[DEBUG] Init lastTrackedHour=%d\n", hourNow);
    return;
  }

  if (hourNow != lastTrackedHour) {
    Serial.printf("[DEBUG] Hour change: %d -> %d\n", lastTrackedHour, hourNow);
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

  // Initial State: Radio OFF to clear any weird hardware states
  WiFi.mode(WIFI_OFF);
  
  // Start connection
  startWifiAttempt(SSID_PRIMARY, PASS_PRIMARY);
}

void loop() {
  ensureConnectivity();
  if (client.connected()) client.loop();

  // snapshot flags safely
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
    
    // Only try to publish if connected; don't force disconnects here
    if (client.connected())
      sendImpulseData(rainfallPerImpulse, currentHourRainfall, tstr);
  }

  maybeSendWeeklySnapshot();
}
