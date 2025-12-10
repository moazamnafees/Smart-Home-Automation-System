/*
  SmartHome_MQTT.ino
  ESP32 + MQTT Smart Home Example
  Features:
    - Reads DHT11 temperature & humidity
    - Reads PIR motion sensor (motion detection)
    - Publishes sensor data to MQTT topics
    - Subscribes to command topics to control two relays (fan, light)
    - Publishes status feedback
    - Reconnect logic + minimal debouncing and rate limiting
  Hardware:
    - ESP32 (Dev board)
    - DHT11 sensor (data pin to DHT_PIN)
    - PIR sensor (digital output to PIR_PIN)
    - Relay modules (to RELAY_FAN_PIN, RELAY_LIGHT_PIN)
  Libraries required:
    - PubSubClient
    - DHT sensor library (by Adafruit)
    - ArduinoJson (optional, not used here)
  Notes:
    - Use MQTT over TLS for production (see README)
    - Adjust sensor read interval, topic names and pins as needed
*/

#include <WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"

// ----------------------------- User Config -----------------------------
const char* WIFI_SSID = "<YOUR_WIFI_SSID>";
const char* WIFI_PASS = "<YOUR_WIFI_PASSWORD>";

// MQTT Broker
const char* MQTT_BROKER = "<MQTT_BROKER_HOST_OR_IP>";
const uint16_t MQTT_PORT = 1883; // 8883 for TLS
const char* MQTT_USER = "<MQTT_USERNAME>";   // optional
const char* MQTT_PASS = "<MQTT_PASSWORD>";   // optional

// Unique client id for broker
const char* CLIENT_ID = "ESP32_SmartHome_01";

// Base topic (change to your desired namespace)
const char* BASE_TOPIC = "home/livingroom";

// Full topics (constructed later)
String topic_temp;
String topic_motion;
String topic_cmd_fan;
String topic_cmd_light;
String topic_status_fan;
String topic_status_light;
String topic_heartbeat;

// Pins (change to your wiring)
const uint8_t DHT_PIN = 15;        // DHT11 data pin
#define DHT_TYPE DHT11
const uint8_t PIR_PIN = 2;        // PIR motion sensor digital output
const uint8_t RELAY_FAN_PIN = 4;  // Relay to control fan
const uint8_t RELAY_LIGHT_PIN = 5; // Relay to control light

// Sampling & publish intervals (ms)
const unsigned long SENSOR_PUBLISH_INTERVAL = 5000; // publish every 5s
const unsigned long HEARTBEAT_INTERVAL = 60000;     // publish heartbeat every 60s
const unsigned long MOTION_DEBOUNCE_MS = 500;       // basic debounce

// ----------------------------- Globals -----------------------------
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
DHT dht(DHT_PIN, DHT_TYPE);

unsigned long lastSensorPublish = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastMotionTime = 0;

bool lastMotionState = false;
bool fanState = false;
bool lightState = false;

// ----------------------------- Helper Functions -----------------------------
void buildTopics() {
  topic_temp = String(BASE_TOPIC) + "/temperature";
  topic_motion = String(BASE_TOPIC) + "/motion";
  topic_cmd_fan = String(BASE_TOPIC) + "/cmd/fan";
  topic_cmd_light = String(BASE_TOPIC) + "/cmd/light";
  topic_status_fan = String(BASE_TOPIC) + "/status/fan";
  topic_status_light = String(BASE_TOPIC) + "/status/light";
  topic_heartbeat = String(BASE_TOPIC) + "/heartbeat";
}

void setRelay(uint8_t pin, bool on) {
  // Assume relays are active HIGH. If your relay is active LOW, invert the logic.
  digitalWrite(pin, on ? HIGH : LOW);
}

void publishStatus(const String &t, const String &payload) {
  if (mqttClient.connected()) {
    mqttClient.publish(t.c_str(), payload.c_str(), true); // retained = true for status
  }
}

void handleCommand(const String &topic, const String &payload) {
  // topic is full topic, payload is message received
  if (topic == topic_cmd_fan) {
    if (payload == "ON" || payload == "1") {
      fanState = true;
    } else if (payload == "OFF" || payload == "0") {
      fanState = false;
    } else {
      // toggle on other messages
      fanState = !fanState;
    }
    setRelay(RELAY_FAN_PIN, fanState);
    publishStatus(topic_status_fan, fanState ? "ON" : "OFF");
  }
  else if (topic == topic_cmd_light) {
    if (payload == "ON" || payload == "1") {
      lightState = true;
    } else if (payload == "OFF" || payload == "0") {
      lightState = false;
    } else {
      lightState = !lightState;
    }
    setRelay(RELAY_LIGHT_PIN, lightState);
    publishStatus(topic_status_light, lightState ? "ON" : "OFF");
  }
}

// MQTT callback
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String t = String(topic);
  String msg = "";
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  msg.trim();
  Serial.printf("[MQTT] Received topic=%s payload=%s\n", t.c_str(), msg.c_str());
  handleCommand(t, msg);
}

void connectToWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
    if (millis() - start > 15000) {
      Serial.println();
      Serial.println("WiFi connect timeout, retrying...");
      start = millis();
    }
  }
  Serial.println();
  Serial.print("WiFi connected. IP: ");
  Serial.println(WiFi.localIP());
}

void mqttReconnect() {
  if (mqttClient.connected()) return;

  Serial.print("[MQTT] Connecting to broker...");
  // Attempt to connect
  bool ok;
  if (strlen(MQTT_USER) > 0) {
    ok = mqttClient.connect(CLIENT_ID, MQTT_USER, MQTT_PASS);
  } else {
    ok = mqttClient.connect(CLIENT_ID);
  }

  if (ok) {
    Serial.println("connected");
    // Subscribe to command topics
    mqttClient.subscribe(topic_cmd_fan.c_str());
    mqttClient.subscribe(topic_cmd_light.c_str());

    // Publish current statuses
    publishStatus(topic_status_fan, fanState ? "ON" : "OFF");
    publishStatus(topic_status_light, lightState ? "ON" : "OFF");
  } else {
    Serial.print("failed, rc=");
    Serial.println(mqttClient.state());
    Serial.println("retrying in 3s");
    delay(3000);
  }
}

// ----------------------------- Setup & Loop -----------------------------
void setup() {
  Serial.begin(115200);
  delay(100);

  // Pins
  pinMode(PIR_PIN, INPUT);
  pinMode(RELAY_FAN_PIN, OUTPUT);
  pinMode(RELAY_LIGHT_PIN, OUTPUT);

  // start with relays OFF
  setRelay(RELAY_FAN_PIN, false);
  setRelay(RELAY_LIGHT_PIN, false);

  dht.begin();
  buildTopics();

  // WiFi + MQTT
  connectToWiFi();
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  // Initial heartbeat/status publish after connect
  mqttReconnect();
  lastSensorPublish = millis();
  lastHeartbeat = millis();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectToWiFi();
  }

  if (!mqttClient.connected()) {
    mqttReconnect();
  }
  mqttClient.loop(); // required to keep connection

  unsigned long now = millis();

  // ------------------ Motion Handling (edge detect + debounce) -------------------
  bool motion = digitalRead(PIR_PIN);
  if (motion != lastMotionState) {
    // Simple debounce/time guard
    if (now - lastMotionTime > MOTION_DEBOUNCE_MS) {
      lastMotionTime = now;
      lastMotionState = motion;
      // Publish motion state immediately
      String mmsg = motion ? "1" : "0";
      publishStatus(topic_motion, mmsg);
      Serial.printf("[SENSOR] Motion: %s\n", motion ? "DETECTED" : "NONE");
    }
  }

  // ------------------ Periodic sensor publish (temperature/humidity) -------------
  if (now - lastSensorPublish >= SENSOR_PUBLISH_INTERVAL) {
    lastSensorPublish = now;

    float temperature = dht.readTemperature(); // Celsius
    float humidity = dht.readHumidity();

    // Check if read failed
    if (isnan(temperature) || isnan(humidity)) {
      Serial.println("[SENSOR] Failed to read DHT sensor");
    } else {
      // Compose payloads as simple text or JSON (here simple)
      char buf[64];
      snprintf(buf, sizeof(buf), "%.2f", temperature);
      publishStatus(topic_temp, String(buf)); // Publish temperature
      Serial.printf("[SENSOR] Temp: %s C  Hum: %.2f %%\n", buf, humidity);
      // You can publish humidity similarly at topic_base/humidity if desired
    }
  }

  // ------------------ Heartbeat (status) -------------
  if (now - lastHeartbeat >= HEARTBEAT_INTERVAL) {
    lastHeartbeat = now;
    String hb = String("{\"ip\":\"") + WiFi.localIP().toString() + String("\",\"uptime_ms\":") + String(millis()) + String("}");
    publishStatus(topic_heartbeat, hb);
  }

  // small delay to allow background tasks
  delay(10);
}
