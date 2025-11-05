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

### 🔹 `test/rainfall/impulse`  *(not retained)*

Real-time message on every tip:

```json
{
  "volume": 0.28,
  "hour_total": 1.40,
  "time": "21:43:02"
}
```

### 🔹 `test/rainfall/hourly`  *(retained)*
```json
{
  "17.10.2025": [
    {
      "0:00": "0",
      "1:00": "0",
      "2:00": "0",
      "3:00": "0",
      "4:00": "0",
      "5:00": "10",
...
      "19:00": "0",
      "20:00": "0",
      "21:00": "0",
      "22:00": "0",
      "23:00": "0"
    }
  ],
  "18.10.2025": [
    {
      "0:00": "0",
      "1:00": "0",
      "2:00": "0",
      "3:00": "0",
...
    }
  ]
}
```
