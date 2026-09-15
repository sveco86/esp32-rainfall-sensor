#include <WiFi.h>
#include <AsyncTCP.h>
#include <HTTPClient.h>
#include <SparkFun_Weather_Meter_Kit_Arduino_Library.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>
#include <ElegantOTA.h>
#include <ESPmDNS.h>
#include <time.h>
#include <esp_task_wdt.h>
#include <esp_system.h>

#include "secrets.h"

/*
 * DOLEZITE - ElegantOTA vyzaduje rucne zapnutie async rezimu,
 * kedze Arduino IDE neumoznuje nastavit build flags:
 *
 * 1. Najdi subor ElegantOTA.h (v priecinku kniznice ElegantOTA,
 *    zvycajne Dokumenty/Arduino/libraries/ElegantOTA/src/).
 * 2. Zmen riadok:
 *      #define ELEGANTOTA_USE_ASYNC_WEBSERVER 0
 *    na:
 *      #define ELEGANTOTA_USE_ASYNC_WEBSERVER 1
 * 3. Uloz a znova skompiluj.
 *
 * Bez tohto kroku kod nepojde skompilovat spolu s AsyncWebServer.
 */

// GPIO pre zrážkomer (podľa fyzického zapojenia z predošlého projektu)
#define RAIN_SENSOR_PIN 27

/*
 * Sieťový názov zariadenia - používa sa pre:
 * - Wi-Fi hostname (posiela sa v DHCP požiadavke, router/UniFi ho
 *   zvyčajne zobrazí ako meno klienta namiesto MAC adresy).
 * - mDNS meno (http://esp32-zrazkomer.local/).
 */
#define DEVICE_HOSTNAME "esp32-zrazkomer"

// Debounce zrážkomera (viď setup() - nastavuje sa cez SparkFun calibration API)
constexpr uint32_t RAIN_DEBOUNCE_MS = 250;

/*
 * Wi-Fi burst ochrana.
 *
 * Vysielanie Wi-Fi rádia (najmä tesne po pripojení) môže indukovať
 * napätie na vodiči k zrážkomeru a spôsobiť falošné impulzy. Po
 * WIFI_NOISE_GUARD_MS od pripojenia sa akékoľvek napočítané impulzy
 * z tohto okna vylúčia z hodinového úhrnu (viď checkWifiNoiseGuard()).
 */
constexpr unsigned long WIFI_NOISE_GUARD_MS = 5000UL; // 5 sekúnd

/*
 * Testovacie príkazy cez Serial (simulácia impulzov zrážkomera bez
 * potreby fyzicky preklápať senzor). Pred ostrým nasadením von
 * odporúčam nastaviť na 0, nech sa test príkazy nedajú omylom spustiť.
 */
#define ENABLE_SERIAL_TEST_COMMANDS 1

// Knižnica vyžaduje aj piny pre senzory vetra,
// hoci tieto senzory nie sú fyzicky pripojené.
#define UNUSED_WIND_SPEED_PIN 4
#define UNUSED_WIND_DIRECTION_PIN 36

// SparkFun Weather Meter Kit
SFEWeatherMeterKit weatherMeterKit(
  UNUSED_WIND_DIRECTION_PIN,
  UNUSED_WIND_SPEED_PIN,
  RAIN_SENSOR_PIN
);

// Web server
AsyncWebServer server(80);

// Interval odosielania: 60 minút
/*
 * Odosielanie údajov je zarovnané na skutočnú hodinu reálneho času
 * (spúšťa sa presne pri minúte 0), nie podľa uplynutého času od
 * štartu zariadenia - viď checkHourlySendTrigger().
 *
 * Ak odoslanie zlyhá, ďalší pokus sa vykoná už o SEND_RETRY_INTERVAL
 * (pár sekúnd), nie až pri ďalšej celej hodine. Po SEND_MAX_QUICK_RETRIES
 * neúspešných rýchlych pokusoch po sebe sa zariadenie vzdá a počká na
 * ďalšiu hodinu, aby zbytočne nezahlcovalo sieť/server pri dlhšom výpadku.
 */
constexpr unsigned long SEND_RETRY_INTERVAL = 15000UL; // 15 sekúnd
constexpr int SEND_MAX_QUICK_RETRIES = 20; // cca 5 minút rýchlych pokusov

// Záložná kontrola Wi-Fi: 30 sekúnd
constexpr unsigned long WIFI_CHECK_INTERVAL = 30000UL;

// Interval logovania stavu haldy: 60 sekúnd
constexpr unsigned long HEAP_CHECK_INTERVAL = 60000UL;

// Ak je Wi-Fi nedostupná dlhšie ako toto, zariadenie sa reštartuje
constexpr unsigned long WIFI_RESTART_THRESHOLD = 30UL * 60UL * 1000UL; // 30 minút

/*
 * Plánovaný denný reštart (čas podľa lokálneho, synchronizovaného času).
 *
 * ENABLE_SCHEDULED_RESTART: 1 = zapnutý, 0 = vypnutý. Užitočné napr.
 * pri diagnostike, ak podozrievate práve tento reštart z toho, že sa
 * po ňom zariadenie neprebralo (viď resetReasonToStr() pri štarte).
 */
#define ENABLE_SCHEDULED_RESTART 0
constexpr int SCHEDULED_RESTART_HOUR = 3;
constexpr int SCHEDULED_RESTART_MINUTE = 1;

// Watchdog timeout
constexpr uint32_t WDT_TIMEOUT_S = 60;

unsigned long previousWiFiCheckMillis = 0;
unsigned long previousHeapCheckMillis = 0;

/*
 * Stav hodinového odosielania.
 *
 * lastSendHourId - identifikátor hodiny (tm_yday*24+tm_hour), pre
 *   ktorú už bol spustený pokus o odoslanie (chráni pred opakovaným
 *   spustením v tej istej minúte).
 * sendPending - true, kým prebieha pokus o odoslanie tejto hodiny
 *   (vrátane prípadných rýchlych retry pokusov).
 * nextSendAttemptMillis - kedy sa má vykonať ďalší pokus.
 */
long lastSendHourId = -1;
bool sendPending = false;
unsigned long nextSendAttemptMillis = 0;
int sendRetryCount = 0;

/*
 * Hodinový úhrn zrážok - oddelený od denného.
 *
 * hourlyBaselineRainfall - hodnota getTotalRainfall() na začiatku
 *   aktuálnej (prebiehajúcej) hodiny. Slúži na výpočet, koľko napršalo
 *   len v tejto hodine (denný súčet mínus tento základ).
 * pendingHourlyRainfall - "zamrazená" hodnota za práve uplynulú hodinu,
 *   ktorá sa posiela na TMEP (aj počas prípadných retry pokusov, nech
 *   sa počas čakania na úspešné odoslanie neprimieša dážď z ďalšej
 *   hodiny).
 */
float hourlyBaselineRainfall = 0.0f;
float pendingHourlyRainfall = 0.0f;

/*
 * Fronta pre prípad, že sa začne nová hodina skôr, než sa dokončí
 * (úspešne alebo vzdaním sa) odosielanie tej predchádzajúcej.
 *
 * queuedHourlyRainfallValid - true, ak čaká hodnota ďalšej hodiny.
 * queuedHourlyRainfall - jej hodnota. Ak by nastalo viackrát po sebe
 *   (viacero hodín čaká, kým sa prvá stále nepodarí odoslať), hodnoty
 *   sa sčítajú do tejto jednej premennej, aby sa nič nestratilo.
 */
bool queuedHourlyRainfallValid = false;
float queuedHourlyRainfall = 0.0f;

/*
 * Stav Wi-Fi burst ochrany (viď WIFI_NOISE_GUARD_MS vyššie).
 *
 * wifiNoiseGuardActive - true, kým beží ochranné okno po pripojení.
 * wifiNoiseGuardUntilMillis - kedy okno končí.
 * rainfallCountsAtGuardStart - počet impulzov na začiatku okna, aby sme
 *   vedeli, koľko ich pribudlo POČAS neho.
 */
bool wifiNoiseGuardActive = false;
unsigned long wifiNoiseGuardUntilMillis = 0;
uint32_t rainfallCountsAtGuardStart = 0;

/*
 * Koľko napršalo od začiatku aktuálnej (ešte prebiehajúcej) hodiny.
 * Na rozdiel od pendingHourlyRainfall (zamrazená hodnota za už
 * uplynulú hodinu) sa toto číslo mení priebežne - užitočné na
 * kontrolu počas testovania, ešte pred tým, ako sa hodina uzavrie.
 */
float getCurrentHourlyRainfall() {
  return weatherMeterKit.getTotalRainfall() - hourlyBaselineRainfall;
}

/*
 * Vyhodnotí Wi-Fi burst ochranné okno po jeho uplynutí.
 *
 * Ak počas okna pribudli nejaké impulzy, sú podozrivé z toho, že ich
 * spôsobila indukcia od Wi-Fi vysielania, nie skutočný dážď. Nedajú sa
 * z internej SparkFun knižnice presne "odpočítať" (knižnica ponúka len
 * úplný reset na 0, nie odčítanie N impulzov), preto namiesto toho
 * posunieme hourlyBaselineRainfall o rovnakú hodnotu - tým sa vylúčia
 * z hodinového úhrnu, ktorý sa posiela na TMEP. Denný úhrn zobrazovaný
 * na stránke zostáva neovplyvnený (je len informatívny, nikam sa
 * neposiela), takže toto je bezpečný kompromis bez rizika straty
 * skutočných dažďových dát.
 */
void checkWifiNoiseGuard() {
  if (!wifiNoiseGuardActive) {
    return;
  }

  if (millis() < wifiNoiseGuardUntilMillis) {
    return;
  }

  wifiNoiseGuardActive = false;

  const uint32_t countsNow = weatherMeterKit.getRainfallCounts();

  if (countsNow > rainfallCountsAtGuardStart) {
    const uint32_t suspiciousCounts = countsNow - rainfallCountsAtGuardStart;
    const float suspiciousRainfall =
      suspiciousCounts * weatherMeterKit.getCalibrationParams().mmPerRainfallCount;

    Serial.printf(
      "[WIFI GUARD] %u podozrivy(ch) impulz(ov) (%.4f mm) tesne po Wi-Fi pripojeni - vylucujem z hodinoveho uhrnu.\n",
      static_cast<unsigned int>(suspiciousCounts),
      suspiciousRainfall
    );

    WebSerial.printf(
      "[WIFI GUARD] %u podozrivy(ch) impulz(ov) (%.4f mm) tesne po Wi-Fi pripojeni - vylucujem z hodinoveho uhrnu.\n",
      static_cast<unsigned int>(suspiciousCounts),
      suspiciousRainfall
    );

    hourlyBaselineRainfall += suspiciousRainfall;
  }
}

// Posledný deň, keď sa vykonal reset zrážok
int lastRainResetDay = -1;

// Posledný deň, keď sa vykonal plánovaný reštart
int lastRestartDay = -1;

bool timeSynchronized = false;

// Sledovanie výpadku Wi-Fi
unsigned long wifiDownSince = 0;
int wifiCheckFailCount = 0;

/*
 * Wi-Fi udalosti.
 *
 * ESP32 automaticky zabezpečuje reconnect cez
 * WiFi.setAutoReconnect(true).
 */
void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_START:
      Serial.println("Wi-Fi stanica spustena.");
      break;

    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("ESP32 sa pripojilo k Wi-Fi access pointu.");

      /*
       * Aktivujeme Wi-Fi burst ochranu - RF aktivita je najsilnejšia
       * práve počas asociácie, čo môže indukovať falošné impulzy na
       * vodiči k zrážkomeru. Viď checkWifiNoiseGuard().
       */
      wifiNoiseGuardActive = true;
      wifiNoiseGuardUntilMillis = millis() + WIFI_NOISE_GUARD_MS;
      rainfallCountsAtGuardStart = weatherMeterKit.getRainfallCounts();

      break;

    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.println("Wi-Fi pripojena.");

      Serial.print("IP adresa: ");
      Serial.println(WiFi.localIP());

      Serial.print("RSSI: ");
      Serial.print(WiFi.RSSI());
      Serial.println(" dBm");

      break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("Wi-Fi spojenie bolo prerusene.");
      Serial.println("ESP32 sa pokusi automaticky znovu pripojit.");
      break;

    default:
      break;
  }
}

void setupWiFi() {
  /*
   * Hostname MUSÍ byť nastavený pred WiFi.mode()/WiFi.begin(),
   * inak sa neuplatní a router uvidí len defaultné meno (napr.
   * "espressif" alebo "ESP32-XXXXXX").
   */
  WiFi.setHostname(DEVICE_HOSTNAME);

  // Režim Wi-Fi klienta
  WiFi.mode(WIFI_STA);

  /*
   * Nezapisovať Wi-Fi konfiguráciu pri každom reconnecte
   * do flash pamäte.
   */
  WiFi.persistent(false);

  /*
   * Vypnúť modem-sleep. Zariadenie je napájané zo siete
   * (nie z batérie), takže úspora energie nie je potrebná
   * a vypnutie zvykne zlepšiť stabilitu a latenciu spojenia.
   */
  WiFi.setSleep(false);

  // Zapnúť automatický reconnect ESP32
  WiFi.setAutoReconnect(true);

  // Registrácia Wi-Fi udalostí
  WiFi.onEvent(WiFiEvent);

  Serial.print("Pripajam sa k Wi-Fi: ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

/*
 * Záložný reconnect.
 *
 * Väčšinu výpadkov rieši WiFi.setAutoReconnect(true).
 * Táto funkcia sa použije ako poistka, ak automatický
 * reconnect z nejakého dôvodu nezaberie.
 *
 * Ak je Wi-Fi nedostupná príliš dlho (WIFI_RESTART_THRESHOLD),
 * vykoná sa plný reštart zariadenia - WiFi stack sa občas
 * dostane do stavu, ktorý vyrieši len reštart.
 */
void checkWiFiConnection() {
  if (WiFi.status() == WL_CONNECTED) {
    wifiDownSince = 0;
    wifiCheckFailCount = 0;
    return;
  }

  if (wifiDownSince == 0) {
    wifiDownSince = millis();
  }

  wifiCheckFailCount++;

  Serial.println("Wi-Fi stale nie je pripojena.");
  WebSerial.println("Wi-Fi stale nie je pripojena.");

  /*
   * Prvý neúspešný check preskočíme - necháme priestor
   * automatickému reconnectu ESP32. Až od druhého checku
   * po sebe zasiahneme manuálne.
   */
  if (wifiCheckFailCount >= 2) {
    Serial.println("Spustam zalozny manualny reconnect.");
    WebSerial.println("Spustam zalozny manualny reconnect.");

    WiFi.disconnect(false);
    WiFi.reconnect();
  }

  const unsigned long downFor = millis() - wifiDownSince;

  if (downFor > WIFI_RESTART_THRESHOLD) {
    Serial.println("Wi-Fi nedostupna viac ako 30 minut, restartujem zariadenie.");
    WebSerial.println("Wi-Fi nedostupna viac ako 30 minut, restartujem zariadenie.");

    delay(200); // necha stihnut odoslat WebSerial spravu
    ESP.restart();
  }
}

unsigned long otaProgressMillis = 0;

void onOTAStart() {
  Serial.println("Zacina OTA aktualizacia.");
  WebSerial.println("Zacina OTA aktualizacia.");
}

void onOTAProgress(size_t current, size_t final) {
  // Nakŕmiť watchdog aj tu - dlhší zápis do flash počas OTA by ho
  // inak mohol nechtiac prepnúť do reštartu uprostred nahrávania.
  esp_task_wdt_reset();

  // Logovat priebeh najviac raz za sekundu.
  if (millis() - otaProgressMillis < 1000) {
    return;
  }

  otaProgressMillis = millis();

  if (final > 0) {
    const unsigned int percent =
      static_cast<unsigned int>((static_cast<uint64_t>(current) * 100ULL) / final);

    Serial.printf(
      "OTA priebeh: %u %% (%u / %u B)\n",
      percent,
      static_cast<unsigned int>(current),
      static_cast<unsigned int>(final)
    );

    WebSerial.printf("OTA priebeh: %u %%\n", percent);
  }
}

void onOTAEnd(bool success) {
  if (success) {
    Serial.println("OTA aktualizacia uspesne dokoncena.");
    WebSerial.println("OTA aktualizacia uspesne dokoncena.");
  } else {
    Serial.println("OTA aktualizacia zlyhala.");
    WebSerial.println("OTA aktualizacia zlyhala.");
  }
}

/*
 * ElegantOTA (async rezim, viz poznamka pri includoch).
 *
 * Nahradza ArduinoOTA - namiesto nahravania z Arduino IDE cez
 * siet sa firmver (.bin, Sketch > Export Compiled Binary) nahra
 * cez webovu stranku http://<IP-adresa>/update.
 *
 * Prihlasovacie udaje su v secrets.h.
 */
void setupElegantOTA() {
  ElegantOTA.begin(&server, OTA_USERNAME, OTA_PASSWORD);

  ElegantOTA.onStart(onOTAStart);
  ElegantOTA.onProgress(onOTAProgress);
  ElegantOTA.onEnd(onOTAEnd);

  Serial.println("ElegantOTA inicializovane, dostupne na /update.");
}

void setupWebServer() {
  WebSerial.begin(&server);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    const float rainfall = weatherMeterKit.getTotalRainfall();

    String html;
    html.reserve(900);

    html += "<!DOCTYPE html>";
    html += "<html lang=\"sk\">";
    html += "<head>";
    html += "<meta charset=\"UTF-8\">";
    html += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
    html += "<meta http-equiv=\"refresh\" content=\"30\">";
    html += "<title>ESP32 zrazkomer</title>";
    html += "</head>";

    html += "<body>";
    html += "<h1>ESP32 zrazkomer</h1>";

    html += "<p><strong>Stav Wi-Fi:</strong> ";

    if (WiFi.status() == WL_CONNECTED) {
      html += "pripojena";
    } else {
      html += "odpojena";
    }

    html += "</p>";

    html += "<p><strong>IP adresa:</strong> ";
    html += WiFi.localIP().toString();
    html += "</p>";

    html += "<p><strong>Pripojene k AP:</strong> ";
    html += WiFi.SSID();
    html += "</p>";

    html += "<p><strong>Wi-Fi RSSI:</strong> ";
    html += String(WiFi.RSSI());
    html += " dBm</p>";

    html += "<p><strong>Denny uhrn:</strong> ";
    html += String(rainfall, 1);
    html += " mm</p>";

    html += "<p><strong>Hodinovy uhrn (prebieha):</strong> ";
    html += String(getCurrentHourlyRainfall(), 2);
    html += " mm</p>";

    html += "<p><strong>Hodinovy uhrn na odoslanie/retry:</strong> ";
    html += sendPending ? String(pendingHourlyRainfall, 2) : "-";
    html += " mm</p>";

    html += "<p><strong>Vo fronte caka:</strong> ";
    html += queuedHourlyRainfallValid ? (String(queuedHourlyRainfall, 2) + " mm") : "nic";
    html += "</p>";

    html += "<p><strong>Volna halda:</strong> ";
    html += String(ESP.getFreeHeap());
    html += " B (minimum: ";
    html += String(ESP.getMinFreeHeap());
    html += " B)</p>";

    html += "<p><strong>Beh od startu:</strong> ";
    html += String(millis() / 1000UL);
    html += " s</p>";

    html += "<p><strong>Dovod posledneho restartu:</strong> ";
    html += resetReasonToStr(esp_reset_reason());
    html += "</p>";

    html += "<p><a href=\"/webserial\">Otvorit WebSerial</a></p>";
    html += "<p><a href=\"/update\">Nahrat novy firmver (ElegantOTA)</a></p>";

    html += "</body>";
    html += "</html>";

    request->send(200, "text/html; charset=UTF-8", html);
  });

  Serial.println("Web server pripraveny.");
}

/*
 * mDNS.
 *
 * ArduinoOTA si mDNS zapinal automaticky (kvoli objaveniu
 * zariadenia v Arduino IDE). ElegantOTA to nerobi, tak si ho
 * zapneme sami - umoznuje pristup cez http://esp32-zrazkomer.local/
 * namiesto pamatania si IP adresy.
 */
void setupMDNS() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("mDNS odlozeny: Wi-Fi nie je pripojena.");
    return;
  }

  if (MDNS.begin(DEVICE_HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
    Serial.println("mDNS spusteny: http://esp32-zrazkomer.local/");
  } else {
    Serial.println("Spustenie mDNS zlyhalo.");
  }
}

void synchronizeTime() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Synchronizacia casu odlozena: Wi-Fi nie je pripojena.");
    return;
  }

  /*
   * Časové pásmo Slovensko:
   *
   * CET  = UTC+1
   * CEST = UTC+2
   *
   * Automatický prechod medzi letným a zimným časom.
   */
  configTzTime(
    "CET-1CEST,M3.5.0/2,M10.5.0/3",
    "pool.ntp.org",
    "time.cloudflare.com",
    "time.google.com"
  );

  Serial.println("Cakam na synchronizaciu casu...");

  struct tm timeInfo;
  const unsigned long startedAt = millis();

  while (
    !getLocalTime(&timeInfo, 1000) &&
    millis() - startedAt < 30000UL
  ) {
    ElegantOTA.loop();
    esp_task_wdt_reset();
    delay(10);
    Serial.print(".");
  }

  Serial.println();

  if (getLocalTime(&timeInfo, 1000)) {
    timeSynchronized = true;

    Serial.println("Cas bol synchronizovany.");

    Serial.printf(
      "Aktualny cas: %04d-%02d-%02d %02d:%02d:%02d\n",
      timeInfo.tm_year + 1900,
      timeInfo.tm_mon + 1,
      timeInfo.tm_mday,
      timeInfo.tm_hour,
      timeInfo.tm_min,
      timeInfo.tm_sec
    );
  } else {
    Serial.println("Synchronizacia casu zlyhala.");
  }
}

/*
 * Odoslanie hodinového úhrnu zrážok na TMEP.
 *
 * Samotné meranie zrážok (počítanie impulzov z tipping-bucket
 * senzora) rieši SparkFun Weather Meter Kit knižnica cez hardvérové
 * prerušenie nastavené v weatherMeterKit.begin() (setup()). Toto
 * počítanie beží nezávisle od Wi-Fi aj od tejto funkcie - zrážky sa
 * teda merajú a spočítavajú aj vtedy, keď je Wi-Fi odpojená alebo
 * čas ešte nie je synchronizovaný.
 *
 * Táto funkcia posiela hodnotu prijatú v parametri rainAmount - je to
 * "zamrazený" prírastok za práve uplynulú hodinu (viď
 * checkHourlySendTrigger()), nie priebežný denný súčet. Vďaka tomu je
 * poslaná hodnota rovnaká pri každom retry pokuse, aj keď medzitým
 * naprší ďalší dážď.
 */
bool sendRainfallToTmep(float rainAmount) {
  /*
   * Bez synchronizovaného času nevieme priradiť namerané zrážky
   * k žiadnej konkrétnej hodine ani dňu (typicky krátko po štarte,
   * kým ešte neprebehla prvá synchronizácia). Odoslanie neskôr, až
   * keď sa čas zosynchronizuje, by tieto zrážky nesprávne pripočítalo
   * k inej (nesprávnej) hodine. Preto ich radšej zahodíme -
   * weatherMeterKit.resetTotalRainfall() - a počkáme na ďalší
   * pokus, až keď už bude čas synchronizovaný.
   */
  if (!timeSynchronized) {
    Serial.println("Cas nie je synchronizovany, namerane zrazky sa zahadzuju (nedaju sa priradit k ziadnej hodine).");
    WebSerial.println("Cas nie je synchronizovany, namerane zrazky sa zahadzuju.");

    weatherMeterKit.resetTotalRainfall();
    hourlyBaselineRainfall = 0.0f;

    return false;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Data sa neodoslali: Wi-Fi nie je pripojena.");
    WebSerial.println("Data sa neodoslali: Wi-Fi nie je pripojena.");
    return false;
  }

  const int rssi = WiFi.RSSI();

  String url;
  url.reserve(160);

  url = String(TMEP_DOMAIN);
  url += "?rain=";
  url += String(rainAmount, 2);
  url += "&rssi=";
  url += String(rssi);

  Serial.println("Odosielam data na TMEP:");
  Serial.println(url);

  WebSerial.println("Odosielam data na TMEP:");
  WebSerial.println(url);

  HTTPClient http;

  http.setConnectTimeout(10000);
  http.setTimeout(10000);

  if (!http.begin(url)) {
    Serial.println("Nepodarilo sa inicializovat HTTP spojenie.");
    WebSerial.println("Nepodarilo sa inicializovat HTTP spojenie.");
    return false;
  }

  const int httpCode = http.GET();
  bool success = false;

  if (httpCode >= 200 && httpCode < 300) {
    Serial.printf(
      "HTTP poziadavka uspesna, kod: %d\n",
      httpCode
    );

    WebSerial.printf(
      "HTTP poziadavka uspesna, kod: %d\n",
      httpCode
    );

    success = true;
  } else if (httpCode > 0) {
    Serial.printf(
      "HTTP server vratil kod: %d\n",
      httpCode
    );

    WebSerial.printf(
      "HTTP server vratil kod: %d\n",
      httpCode
    );
  } else {
    const String errorMessage = http.errorToString(httpCode);

    Serial.printf(
      "HTTP poziadavka zlyhala: %s\n",
      errorMessage.c_str()
    );

    WebSerial.printf(
      "HTTP poziadavka zlyhala: %s\n",
      errorMessage.c_str()
    );
  }

  http.end();

  return success;
}

/*
 * Spúšťač hodinového odosielania.
 *
 * Spustí nový pokus o odoslanie presne pri minúte 0 reálneho,
 * synchronizovaného času - nie podľa uplynutého času od štartu.
 * Guard cez lastSendHourId zabráni opakovanému spusteniu počas
 * tej istej minúty (funkcia sa volá pri každom priechode loop()).
 *
 * hourlyBaselineRainfall sa aktualizuje VŽDY (aby getCurrentHourlyRainfall()
 * spoľahlivo sledovalo aktuálnu hodinu bez ohľadu na stav odosielania).
 * Naproti tomu pendingHourlyRainfall sa prepíše len vtedy, ak už
 * neprebieha odosielanie predchádzajúcej hodiny - inak sa nová hodnota
 * zaradí do fronty (viď queuedHourlyRainfall) a počká, kým sa
 * predchádzajúca hodina uzavrie.
 */
void checkHourlySendTrigger() {
  if (!timeSynchronized) {
    return;
  }

  struct tm timeInfo;

  if (!getLocalTime(&timeInfo, 100)) {
    return;
  }

  const long currentHourId =
    static_cast<long>(timeInfo.tm_yday) * 24L + timeInfo.tm_hour;

  if (
    timeInfo.tm_min == 0 &&
    currentHourId != lastSendHourId
  ) {
    lastSendHourId = currentHourId;

    const float currentTotal = weatherMeterKit.getTotalRainfall();
    const float finishedHourRainfall = currentTotal - hourlyBaselineRainfall;
    hourlyBaselineRainfall = currentTotal;

    Serial.printf("Hodinovy uhrn za poslednu hodinu: %.4f mm\n", finishedHourRainfall);
    WebSerial.printf("Hodinovy uhrn za poslednu hodinu: %.4f mm\n", finishedHourRainfall);

    if (sendPending) {
      /*
       * Predchádzajúca hodina sa ešte stále odosiela/retryuje.
       * Jej hodnotu nemeníme - túto novú hodinu zaradíme do fronty.
       */
      if (queuedHourlyRainfallValid) {
        queuedHourlyRainfall += finishedHourRainfall;
      } else {
        queuedHourlyRainfall = finishedHourRainfall;
        queuedHourlyRainfallValid = true;
      }

      Serial.println("Predchadzajuce odoslanie este stale prebieha, nova hodina caka vo fronte.");
      WebSerial.println("Predchadzajuce odoslanie este stale prebieha, nova hodina caka vo fronte.");
    } else {
      pendingHourlyRainfall = finishedHourRainfall;
      sendPending = true;
      sendRetryCount = 0;
      nextSendAttemptMillis = millis();
    }
  }
}

/*
 * Ak čaká vo fronte hodnota ďalšej hodiny (viď checkHourlySendTrigger),
 * presunie ju do pendingHourlyRainfall a spustí pre ňu nový cyklus
 * odosielania. Volá sa vždy, keď sa predchádzajúci pokus uzavrie -
 * úspechom, alebo vzdaním sa po SEND_MAX_QUICK_RETRIES pokusoch.
 */
void promoteQueuedHourlyRainfall() {
  if (!queuedHourlyRainfallValid) {
    return;
  }

  pendingHourlyRainfall = queuedHourlyRainfall;
  queuedHourlyRainfallValid = false;
  queuedHourlyRainfall = 0.0f;

  sendPending = true;
  sendRetryCount = 0;
  nextSendAttemptMillis = millis();

  Serial.println("Odosielam hodinu, ktora cakala vo fronte.");
  WebSerial.println("Odosielam hodinu, ktora cakala vo fronte.");
}

/*
 * Spracuje prebiehajúci pokus o odoslanie (vrátane rýchlych retry).
 *
 * Pri zlyhaní naplánuje ďalší pokus o SEND_RETRY_INTERVAL. Po
 * SEND_MAX_QUICK_RETRIES neúspešných pokusoch sa vzdá až do ďalšej
 * hodiny (ďalší pokus vytvorí checkHourlySendTrigger()).
 */
void processPendingSend() {
  if (!sendPending) {
    return;
  }

  if (millis() < nextSendAttemptMillis) {
    return;
  }

  if (sendRainfallToTmep(pendingHourlyRainfall)) {
    sendPending = false;
    sendRetryCount = 0;
    promoteQueuedHourlyRainfall();
    return;
  }

  sendRetryCount++;

  if (sendRetryCount >= SEND_MAX_QUICK_RETRIES) {
    Serial.println("Prekroceny pocet rychlych pokusov o odoslanie, cakam do dalsej hodiny.");
    WebSerial.println("Prekroceny pocet rychlych pokusov o odoslanie, cakam do dalsej hodiny.");

    sendPending = false;
    promoteQueuedHourlyRainfall();
  } else {
    nextSendAttemptMillis = millis() + SEND_RETRY_INTERVAL;
  }
}

void resetRainfallAtMidnight() {
  if (!timeSynchronized) {
    return;
  }

  struct tm timeInfo;

  if (!getLocalTime(&timeInfo, 100)) {
    timeSynchronized = false;
    return;
  }

  /*
   * Reset raz počas prvej minúty nového dňa.
   *
   * Kontrola tm_yday zabráni opakovanému resetu
   * pri každom priechode loop().
   */
  if (
    timeInfo.tm_hour == 0 &&
    timeInfo.tm_min == 0 &&
    timeInfo.tm_yday != lastRainResetDay
  ) {
    weatherMeterKit.resetTotalRainfall();
    hourlyBaselineRainfall = 0.0f;
    lastRainResetDay = timeInfo.tm_yday;

    Serial.println("Denny uhrn zrazok bol vynulovany.");
    WebSerial.println("Denny uhrn zrazok bol vynulovany.");
  }
}

/*
 * Plánovaný denný reštart.
 *
 * Aj pri dobre napísanom kóde sa na zariadeniach bežiacich
 * mesiace naostro postupne prejaví drobná fragmentácia haldy
 * alebo iný pomalý "leak". Pravidelný reštart v čase, keď to
 * nevadí (3:00 v noci, mimo prevádzkovo dôležitého okna), je
 * lacná poistka.
 */
void scheduledDailyRestart() {
#if ENABLE_SCHEDULED_RESTART
  if (!timeSynchronized) {
    return;
  }

  struct tm timeInfo;

  if (!getLocalTime(&timeInfo, 100)) {
    return;
  }

  if (
    timeInfo.tm_hour == SCHEDULED_RESTART_HOUR &&
    timeInfo.tm_min == SCHEDULED_RESTART_MINUTE &&
    timeInfo.tm_yday != lastRestartDay
  ) {
    lastRestartDay = timeInfo.tm_yday;

    Serial.println("Planovany denny restart zariadenia.");
    WebSerial.println("Planovany denny restart zariadenia.");

    delay(200); // necha stihnut odoslat WebSerial spravu
    ESP.restart();
  }
#endif // ENABLE_SCHEDULED_RESTART
}

void logHeapStatus() {
  const uint32_t freeHeap = ESP.getFreeHeap();
  const uint32_t minFreeHeap = ESP.getMinFreeHeap();

  Serial.printf(
    "Volna halda: %u B (minimum od startu: %u B)\n",
    freeHeap,
    minFreeHeap
  );

  WebSerial.printf(
    "Volna halda: %u B (minimum od startu: %u B)\n",
    freeHeap,
    minFreeHeap
  );
}

#if ENABLE_SERIAL_TEST_COMMANDS

/*
 * Softvérovo simuluje jeden impulz zrážkomera.
 *
 * Krátko strhne pin do LOW a pustí ho späť do INPUT_PULLUP - presne to
 * isté, čo urobí fyzický reed switch senzora pri jednom preklopení
 * (moment closure). SparkFun knižnica reálne používa INPUT_PULLUP +
 * RISING interrupt (overené priamo v jej zdrojovom kóde) - simulácia
 * túto RISING hranu vytvorí pri návrate z LOW späť na HIGH, takže
 * funguje správne aj napriek tomu.
 *
 * Odstup medzi jednotlivými simulovanými impulzmi rešpektuje interné
 * debounce v SparkFun knižnici (RAIN_DEBOUNCE_MS) - ak pošlete impulzy
 * príliš rýchlo za sebou, časť z nich sa odfiltruje rovnako, ako by sa
 * to stalo pri reálnom senzore.
 */
void simulateRainfallImpulse() {
  pinMode(RAIN_SENSOR_PIN, OUTPUT);
  digitalWrite(RAIN_SENSOR_PIN, LOW);
  delayMicroseconds(200);
  digitalWrite(RAIN_SENSOR_PIN, HIGH);
  pinMode(RAIN_SENSOR_PIN, INPUT_PULLUP);
}

/*
 * Spracuje testovacie príkazy zadané cez Serial Monitor (Enter na konci):
 *   tip          - simuluje jeden impulz zrážkomera
 *   tip <pocet>  - simuluje zadaný počet impulzov za sebou
 *   status       - vypíše aktuálny denný úhrn a stav Wi-Fi/času
 */
void checkSerialTestCommands() {
  if (!Serial.available()) {
    return;
  }

  String line = Serial.readStringUntil('\n');
  line.trim();

  if (line.length() == 0) {
    return;
  }

  if (line.startsWith("tip")) {
    int count = 1;
    String arg = line.substring(3);
    arg.trim();

    if (arg.length() > 0) {
      count = arg.toInt();

      if (count <= 0) {
        count = 1;
      }
    }

    for (int i = 0; i < count; i++) {
      simulateRainfallImpulse();
      delay(RAIN_DEBOUNCE_MS + 20); // odstup nad novym debounce prahom
    }

    Serial.printf(
      "Simulovanych impulzov: %d. Aktualny denny uhrn: %.4f mm\n",
      count,
      weatherMeterKit.getTotalRainfall()
    );

    WebSerial.printf(
      "Simulovanych impulzov: %d. Aktualny denny uhrn: %.4f mm\n",
      count,
      weatherMeterKit.getTotalRainfall()
    );

    return;
  }

  if (line == "status") {
    Serial.printf(
      "Denny uhrn: %.4f mm | Hodinovy uhrn (prebieha): %.4f mm | Odosielanie: %s (%.4f mm) | Vo fronte: %s\n",
      weatherMeterKit.getTotalRainfall(),
      getCurrentHourlyRainfall(),
      sendPending ? "prebieha/retry" : "necinne",
      pendingHourlyRainfall,
      queuedHourlyRainfallValid ? String(queuedHourlyRainfall, 4).c_str() : "nic"
    );

    Serial.printf(
      "Wi-Fi: %s (%s) | Cas synchronizovany: %s\n",
      WiFi.status() == WL_CONNECTED ? "pripojena" : "odpojena",
      WiFi.SSID().c_str(),
      timeSynchronized ? "ano" : "nie"
    );
    return;
  }

  Serial.println("Neznamy prikaz. Dostupne: 'tip', 'tip <pocet>', 'status'.");
}

#endif // ENABLE_SERIAL_TEST_COMMANDS

/*
 * Preloží dôvod posledného reštartu/resetu do čitateľného textu.
 *
 * Kľúčové pre diagnostiku problémov okolo plánovaného reštartu -
 * BROWNOUT znamená problém s napájaním (Wi-Fi rádio pri štarte
 * krátkodobo odoberá viac prúdu), TASK_WDT/INT_WDT/WDT znamená, že
 * niečo v kóde viselo dlhšie ako watchdog timeout, SW je čistý
 * očakávaný reštart cez ESP.restart().
 */
const char* resetReasonToStr(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:   return "POWERON (normalne zapnutie/napajanie)";
    case ESP_RST_EXT:       return "EXT (externy reset pin)";
    case ESP_RST_SW:        return "SW (softverovy ESP.restart())";
    case ESP_RST_PANIC:     return "PANIC (crash / vynimka v kode)";
    case ESP_RST_INT_WDT:   return "INT_WDT (interny watchdog)";
    case ESP_RST_TASK_WDT:  return "TASK_WDT (nas task watchdog - nieco viselo)";
    case ESP_RST_WDT:       return "WDT (iny watchdog)";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP (prebudenie zo spanku)";
    case ESP_RST_BROWNOUT:  return "BROWNOUT (podpatie napajania!)";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "NEZNAMY dovod";
  }
}

/*
 * Hardvérový/task watchdog.
 *
 * Ak sa loop() alebo setup() niekde zablokujú (visiaca
 * knižnica, HTTP zásobník, callback...), zariadenie sa po
 * WDT_TIMEOUT_S sekundách samo reštartuje namiesto toho,
 * aby zostalo navždy visieť.
 *
 * POZOR na verziu arduino-esp32 core:
 * - core 3.x (IDF 5.x) - použi kód nižšie (esp_task_wdt_config_t).
 * - core 2.x (staršie) - nahraď blok za:
 *     esp_task_wdt_init(WDT_TIMEOUT_S, true);
 *     esp_task_wdt_add(NULL);
 */
void setupWatchdog() {
  esp_task_wdt_config_t wdtConfig = {
    .timeout_ms = WDT_TIMEOUT_S * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };

  esp_task_wdt_reconfigure(&wdtConfig);
  esp_task_wdt_add(NULL);

  Serial.println("Watchdog inicializovany.");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("Spustam ESP32 zrazkomer...");

  Serial.printf("Dovod posledneho restartu: %s\n", resetReasonToStr(esp_reset_reason()));

  // Watchdog čo najskôr, nech chráni aj zvyšok setup().
  setupWatchdog();

  /*
   * Predĺžený debounce zrážkomera (250 ms namiesto štandardných
   * 100 ms) - znižuje riziko falošných impulzov z odskoku kontaktu
   * reed switchu. Musí sa nastaviť pred begin() (aj keď poradie tu
   * technicky nie je kritické, keďže begin() len nastaví piny a
   * pripojí prerušenie, kalibrácia je nezávislá).
   */
  SFEWeatherMeterKitCalibrationParams calParams = weatherMeterKit.getCalibrationParams();
  calParams.minMillisPerRainfall = RAIN_DEBOUNCE_MS;
  weatherMeterKit.setCalibrationParams(calParams);

  /*
   * Inicializácia SparkFun Weather Meter Kit.
   *
   * Musí zostať, pretože nastaví obsluhu impulzov
   * zo zrážkomera.
   */
  weatherMeterKit.begin();

  setupWiFi();

  /*
   * Počkáme maximálne 20 sekúnd na prvé Wi-Fi pripojenie.
   * Ak sa nepodarí, program ďalej beží a Wi-Fi sa bude
   * pripájať automaticky.
   */
  const unsigned long wifiStartedAt = millis();

  while (
    WiFi.status() != WL_CONNECTED &&
    millis() - wifiStartedAt < 20000UL
  ) {
    esp_task_wdt_reset();
    delay(250);
    Serial.print(".");
  }

  Serial.println();

  setupMDNS();
  setupWebServer();
  setupElegantOTA();

  // server.begin() musi ist az po ElegantOTA.begin().
  server.begin();
  Serial.println("Web server spusteny.");

  synchronizeTime();

  Serial.println("ESP32 zrazkomer je spusteny.");
  WebSerial.println("ESP32 zrazkomer je spusteny.");
}

void loop() {
  const unsigned long currentMillis = millis();

  // Nakŕmiť watchdog na začiatku každého priechodu loop().
  esp_task_wdt_reset();

  ElegantOTA.loop();

#if ENABLE_SERIAL_TEST_COMMANDS
  checkSerialTestCommands();
#endif

  /*
   * Vyhodnotenie Wi-Fi burst ochrany (viď WiFiEvent() a komentár pri
   * funkcii). Musí bežať pri každom priechode loop(), nie len raz za
   * 30 s, aby okno zachytilo presne včas.
   */
  checkWifiNoiseGuard();

  /*
   * Záložná kontrola Wi-Fi každých 30 sekúnd.
   *
   * Primárne reconnect zabezpečuje ESP32 automaticky.
   */
  if (
    currentMillis - previousWiFiCheckMillis >= WIFI_CHECK_INTERVAL
  ) {
    previousWiFiCheckMillis = currentMillis;

    checkWiFiConnection();

    /*
     * Ak sa po výpadku Wi-Fi čas ešte nesynchronizoval,
     * vykonáme nový pokus.
     */
    if (
      WiFi.status() == WL_CONNECTED &&
      !timeSynchronized
    ) {
      synchronizeTime();
    }
  }

  /*
   * Odosielanie údajov - zarovnané na skutočnú hodinu (viď funkcie
   * vyššie), nie podľa uplynutého času od štartu.
   */
  checkHourlySendTrigger();
  processPendingSend();

  // Logovanie stavu haldy každých 60 sekúnd
  if (
    currentMillis - previousHeapCheckMillis >= HEAP_CHECK_INTERVAL
  ) {
    previousHeapCheckMillis = currentMillis;

    logHeapStatus();
  }

  resetRainfallAtMidnight();
  scheduledDailyRestart();

  delay(10);
}
