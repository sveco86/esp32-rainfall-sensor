# 🌧️ ESP32 Rainfall Sensor (Tipping Bucket + MQTT)

A robust, Wi-Fi-connected **ESP32 rainfall monitor** based on the **DFRobot SEN0575 tipping-bucket rain gauge**.  
The project measures rainfall accurately, handles Wi-Fi reconnects and DST automatically, and publishes live and hourly rainfall data to an MQTT broker for visualization (Home Assistant, Node-RED, Grafana, etc.).

> ⚠️ **Project status:** This project is currently in the **testing and calibration phase**.  
> Hardware and software are functional, but behavior under long-term outdoor conditions and MQTT performance are still being validated.  
> Contributions and feedback are welcome!

---

## 📦 Features

✅ **Accurate rainfall measurement**  
- Uses **DFRobot SEN0575** tipping-bucket rain gauge (reed switch output).  
- Detects every tip via an interrupt on ESP32 GPIO.  
- Each tip equals a configurable rainfall volume (default `0.28 mm`).  

✅ **Time-aware data logging**  
- Uses ESP32 SNTP with automatic **CET/CEST DST** correction.  
- Keeps a rolling log of **7 days × 24 hours** rainfall totals.  

✅ **MQTT integration**  
- `test/rainfall/impulse` → Real-time impulses with current-hour total.  
- `test/rainfall/hourly` → Retained 7-day snapshot with all hourly values.  
- Messages use the most reliable delivery supported by PubSubClient.  

✅ **Offline recovery**  
- Continues counting rainfall while offline.  
- Sends all accumulated data once Wi-Fi or MQTT reconnects.  

✅ **Resilient Wi-Fi**  
- Automatically switches between primary and backup SSID.  
- Detects stuck connection attempts and resets the interface.

✅ **OTA updates** 
- Used ElegantOTA library


---

## 🧰 Hardware

| Component | Model / Description | Connection |
|------------|--------------------|-------------|
| **ESP32 DevKit** | ESP-WROOM-32 or WROVER module | Main controller |
| **Rain gauge** | DFRobot SEN0575 tipping-bucket sensor | `Signal → GPIO27`, `GND → GND` |
| **Power** | 5 V USB or regulated 5 V supply | via ESP32 Vin/USB |

🧩 **SEN0575 notes**
- Output is a simple **reed switch** (digital on/off).  
- Internal magnet closes contact once per bucket tip (≈ 0.28 mm of rain).  
- ESP32 pin uses `INPUT_PULLUP`, so **no external resistor** is required.  
- Ensure the cable and junctions are **weatherproof** for outdoor use.

---

## ⚙️ MQTT Topics & Payloads

### 🔹 `test/rainfall/impulse`

Real-time message on every tip:

```json
{
  "volume": 0.28,
  "hour_total": 1.40,
  "time": "2025-11-23T21:43:02"
}
```

### 🔹 `test/rainfall/hourly`

```json
{
  "2025-10-17": [
    {
      "00:59:59": "0",
      "01:59:59": "0",
      "02:59:59": "0",
      "03:59:59": "0",
      "04:59:59": "0",
      "05:59:59": "10",
      "06:59:59": "15",
      "07:59:59": "5",
      "08:59:59": "0",
      "09:59:59": "0",
      "10:59:59": "0",
      "11:59:59": "0",
      "12:59:59": "0",
      "13:59:59": "0",
      "14:59:59": "0",
      "15:59:59": "0",
      "16:59:59": "0",
      "17:59:59": "5",
      "18:59:59": "10",
      "19:59:59": "0",
      "20:59:59": "0",
      "21:59:59": "0",
      "22:59:59": "0",
      "23:59:59": "0"
    }
  ],
  "2025-10-18": [
    {
      "00:59:59": "0",
      "01:59:59": "0",
      "02:59:59": "0",
      "03:59:59": "0",
      "04:59:59": "0",
      "05:59:59": "10",
      "06:59:59": "15",
      "07:59:59": "5",
      "08:59:59": "0",
      "09:59:59": "0",
      "10:59:59": "0",
      "11:59:59": "0"
    }
  ]
}
```
