# Smart-Home-Automation-System
# Smart Home Automation — ESP32 + MQTT

**Short description**  
ESP32-based Smart Home Automation example using MQTT.  
This project demonstrates reading sensors (DHT11, PIR) and controlling actuators (relays for fan and light) via MQTT topics. It is designed as a compact, secure, and extensible template for IoT projects.

---

## Features
- Temperature (DHT11) reading and publishing
- Motion detection (PIR) and publishing
- Remote control of Fan and Light via MQTT command topics
- Status/topic feedback with retained messages
- Reconnect logic for Wi-Fi and MQTT
- Easily extensible to more sensors/actuators

---

## Hardware
- **ESP32** development board (any common dev board)
- **DHT11** temperature & humidity sensor
- **PIR** motion sensor (e.g., HC-SR501)
- **2x Relay modules** (for Fan and Light) — or solid-state switch
- **Power**: 5V USB for ESP32 (separate power for relays/motor as required)
- **Optional**: Level shifting or transistor driver for relays if needed

### Suggested wiring
- `DHT_PIN`   -> ESP32 GPIO 15 (data) (with 10K pull-up if needed)
- `PIR_PIN`   -> ESP32 GPIO 2 (digital output)
- `RELAY_FAN_PIN`  -> ESP32 GPIO 4 (relay input)
- `RELAY_LIGHT_PIN`-> ESP32 GPIO 5 (relay input)
- VCC/GND -> power appropriately (observe relay voltage)

> **Warning:** If switching mains AC loads, ensure you follow electrical safety practices and use proper relay modules / opto-isolation. Prefer mains-rated relays and protective enclosures.

---

## Software / Libraries
- Arduino core for ESP32 (install via Boards Manager)
- [PubSubClient](https://github.com/knolleary/pubsubclient)
- [DHT sensor library (Adafruit)](https://github.com/adafruit/DHT-sensor-library)
- (Optional) [ArduinoJson] for JSON payloads

Install libraries in Arduino IDE or PlatformIO before compiling.

---

## Topics & Payloads
Base topic: `home/livingroom` (configurable in code)

| Purpose | Topic | Payload Example |
|---------|-------|-----------------|
| Temperature publish | `home/livingroom/temperature` | `26.45` |
| Motion publish | `home/livingroom/motion` | `1` or `0` |
| Command — Fan | `home/livingroom/cmd/fan` | `ON` / `OFF` / `TOGGLE` |
| Command — Light | `home/livingroom/cmd/light` | `ON` / `OFF` / `TOGGLE` |
| Status — Fan | `home/livingroom/status/fan` | `ON` / `OFF` (retained) |
| Status — Light | `home/livingroom/status/light` | `ON` / `OFF` (retained) |
| Heartbeat | `home/livingroom/heartbeat` | JSON with IP/uptime |

---

## Setup & Usage

1. Clone repo to your computer.
2. Open `SmartHome_MQTT.ino` in Arduino IDE (or import to PlatformIO).
3. Edit Wi-Fi and MQTT credentials at the top of the sketch:
   ```cpp
   const char* WIFI_SSID = "<YOUR_WIFI_SSID>";
   const char* WIFI_PASS = "<YOUR_WIFI_PASSWORD>";
   const char* MQTT_BROKER = "<MQTT_BROKER_HOST_OR_IP>";
   const uint16_t MQTT_PORT = 1883; // use 8883 for secure TLS
   const char* MQTT_USER = "<MQTT_USERNAME>";
   const char* MQTT_PASS = "<MQTT_PASSWORD>";
