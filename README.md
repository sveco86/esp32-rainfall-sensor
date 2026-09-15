# ESP32 Rainfall Sensor

A Wi-Fi-connected ESP32 rain gauge built around the **SparkFun Weather Meter Kit**'s tipping-bucket sensor. It measures hourly rainfall, reports it to [TMEP.cz](https://www.tmep.cz/), and is hardened for long-term unattended outdoor operation (watchdog, automatic Wi-Fi recovery, scheduled restarts, retry-with-backoff reporting).

> ⚠️ **Project status:** actively developed and under test. The current codebase replaces an earlier MQTT/DFRobot-based prototype that lived in this repo.

## Features

- **Rainfall measurement** via the official [SparkFun Weather Meter Kit Arduino Library](https://github.com/sparkfun/SparkFun_Weather_Meter_Kit_Arduino_Library), using hardware interrupts — counting runs independently of Wi-Fi and the rest of the firmware.
- **250 ms debounce** (raised from the library's 100 ms default) to reduce false triggers from reed-switch contact bounce.
- **Wi-Fi noise guard** — discards any rain pulses detected in the first 5 seconds after each Wi-Fi (re)connection, since RF transmission can occasionally induce false pulses on the sensor wiring.
- **Hourly, clock-aligned reporting** — sends the exact rainfall for each completed hour (not a running daily total) to TMEP, triggered precisely at `:00` of every hour rather than on a rolling timer from boot.
- **Retry with backoff** — a failed send retries every 15 s (up to 20 attempts, ~5 minutes) instead of waiting for the next hour; a one-slot queue makes sure a new hour's data is never lost or merged incorrectly if the previous hour is still retrying.
- **Data-integrity guard** — if the system clock isn't yet synchronized (e.g. shortly after boot), any rain measured in that window is discarded rather than being attributed to the wrong hour.
- **Daily total** is tracked separately and shown on the status page, purely for local reference (it is never transmitted).
- **Wi-Fi resilience** — automatic reconnect, a manual reconnect fallback after repeated failures, and a full device restart if Wi-Fi stays down for more than 30 minutes.
- **Hardware/task watchdog** (60 s) with reset-reason logging (`esp_reset_reason()`) on every boot, so power/brownout issues are distinguishable from software hangs.
- **Optional scheduled daily restart** (03:01 by default) as a cheap defence against long-uptime memory fragmentation — can be disabled with a single build-time flag.
- **OTA updates** via [ElegantOTA](https://github.com/ayushsharma82/ElegantOTA) (async mode) at `http://<device-ip>/update`, protected by a username/password.
- **Remote log viewer** via [WebSerial](https://github.com/ayushsharma82/WebSerial) at `http://<device-ip>/webserial`.
- **mDNS + DHCP hostname** — reachable as `esp32-zrazkomer.local`, and reports the same name to your router/DHCP server (shows up by name in most router/UniFi client lists).
- **Status web page** (`/`) showing Wi-Fi state, SSID, RSSI, daily/hourly rainfall, free heap, uptime, and last restart reason.
- **Built-in Serial test commands** for bench-testing without a physical sensor (see below).

## Hardware

- ESP32 dev board (tested with the classic `esp32:esp32:esp32` / "ESP32 Dev Module").
- [SparkFun Weather Meter Kit](https://www.sparkfun.com/sparkfun-weather-meter-kit.html) rain gauge (tipping-bucket reed switch), wired to a GPIO with `INPUT_PULLUP` (default: GPIO 27).
- No wind sensors are physically connected; the library still requires pins to be defined for them, so unused GPIOs are assigned (defaults: GPIO 4 and GPIO 36).

## Libraries required

Install via Arduino Library Manager unless noted otherwise:

| Library | Notes |
|---|---|
| [SparkFun Weather Meter Kit Arduino Library](https://github.com/sparkfun/SparkFun_Weather_Meter_Kit_Arduino_Library) | Rain pulse counting |
| [ESP Async WebServer](https://github.com/ESP32Async/ESPAsyncWebServer) (ESP32Async fork) | Web server |
| [AsyncTCP](https://github.com/ESP32Async/AsyncTCP) (ESP32Async fork) | **Must match** the ESPAsyncWebServer fork/version, or compilation fails |
| [WebSerial](https://github.com/ayushsharma82/WebSerial) | Remote log viewer |
| [ElegantOTA](https://github.com/ayushsharma82/ElegantOTA) | Web-based OTA updates |
| `WiFi`, `HTTPClient`, `ESPmDNS`, `esp_task_wdt`, `esp_system` | Bundled with the ESP32 Arduino core |

### ElegantOTA async mode

ElegantOTA needs async mode enabled manually, since Arduino IDE doesn't support build flags:

1. Open `ElegantOTA.h` inside the ElegantOTA library folder.
2. Change `#define ELEGANTOTA_USE_ASYNC_WEBSERVER 0` to `1`.
3. Save and recompile.

### Partition scheme

Use **Tools → Partition Scheme → "Minimal SPIFFS (1.9MB APP with OTA)"** on a 4MB board. The default scheme's ~1.25 MB app partition is too tight for this firmware's OTA image.

## Configuration

Copy the template below into a `secrets.h` file next to the `.ino` (kept out of version control):

```cpp
#pragma once

#define WIFI_SSID     "your-ssid"
#define WIFI_PASSWORD "your-password"

#define TMEP_DOMAIN "http://your-domain.tmep.cz/"

#define OTA_USERNAME "admin"
#define OTA_PASSWORD "change-me"
```

## Usage

| Endpoint / command | Description |
|---|---|
| `http://esp32-zrazkomer.local/` (or device IP) | Status page |
| `http://esp32-zrazkomer.local/webserial` | Live remote log |
| `http://esp32-zrazkomer.local/update` | OTA firmware upload (requires login) |
| Serial `tip` | Simulates one rain gauge pulse (for bench testing) |
| Serial `tip <count>` | Simulates multiple pulses in a row |
| Serial `status` | Prints current rainfall totals, send/queue state, Wi-Fi and time-sync status |

Serial test commands can be disabled for production by setting `ENABLE_SERIAL_TEST_COMMANDS` to `0`.

## Key configuration constants

| Constant | Default | Purpose |
|---|---|---|
| `RAIN_SENSOR_PIN` | `27` | GPIO wired to the rain gauge reed switch |
| `RAIN_DEBOUNCE_MS` | `250` | Minimum time between counted pulses |
| `WIFI_NOISE_GUARD_MS` | `5000` | Window after Wi-Fi connect during which pulses are treated as suspect |
| `SEND_RETRY_INTERVAL` | `15000` | Retry delay after a failed send (ms) |
| `SEND_MAX_QUICK_RETRIES` | `20` | Retries before giving up until the next hour |
| `WIFI_RESTART_THRESHOLD` | 30 minutes | Wi-Fi downtime before a full device restart |
| `ENABLE_SCHEDULED_RESTART` | `1` | `0` disables the daily restart entirely |
| `SCHEDULED_RESTART_HOUR` / `_MINUTE` | `3` / `1` | Time of the daily restart |
| `WDT_TIMEOUT_S` | `60` | Watchdog timeout |
