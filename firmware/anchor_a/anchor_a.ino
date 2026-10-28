/*
 * ESP32 Indoor Positioning System - Device A (Fixed Wi-Fi Anchor A)
 *
 * Hardware: Standard ESP32 Development Board
 * Role: Fixed Wi-Fi Anchor broadcasting SSID "INDOOR_ANCHOR_A"
 * Coordinates: (0.0 m, 0.0 m) by default (configurable in backend)
 * No GPS, IMU, or LoRa hardware required.
 */

#include <WiFi.h>

const char* ANCHOR_SSID = "INDOOR_ANCHOR_A";
const char* ANCHOR_PASSWORD = "";  // Open network for passive beaconing
const int WIFI_CHANNEL = 1;        // Dedicated Wi-Fi channel
const int MAX_CLIENTS = 4;
const int LED_PIN = 2;             // Onboard blue LED on most ESP32 dev boards

unsigned long lastHeartbeat = 0;
const unsigned long HEARTBEAT_INTERVAL_MS = 2000;

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Serial.println("\n==============================================");
  Serial.println("  ESP32 INDOOR POSITIONING - ANCHOR A");
  Serial.println("==============================================");

  // Configure Wi-Fi SoftAP
  WiFi.mode(WIFI_AP);
  
  // Set fixed maximum Wi-Fi TX power for consistent RSSI calibration
  WiFi.setTxPower(WIFI_POWER_19_5dBm);

  bool apStarted = WiFi.softAP(ANCHOR_SSID, ANCHOR_PASSWORD, WIFI_CHANNEL, 0, MAX_CLIENTS);

  if (apStarted) {
    Serial.printf("[OK] SoftAP started successfully.\n");
    Serial.printf("     SSID: %s\n", ANCHOR_SSID);
    Serial.printf("     Channel: %d\n", WIFI_CHANNEL);
    Serial.printf("     IP: %s\n", WiFi.softAPIP().toString().c_str());
    Serial.printf("     MAC: %s\n", WiFi.softAPmacAddress().c_str());
  } else {
    Serial.println("[ERROR] Failed to start SoftAP!");
  }
}

void loop() {
  unsigned long now = millis();

  // Periodic heartbeat diagnostic
  if (now - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeat = now;
    
    // Toggle status LED
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    Serial.printf("[HEARTBEAT] Anchor A Active | Uptime: %lu s | Stations: %d\n",
                  now / 1000,
                  WiFi.softAPgetStationNum());
  }

  delay(50);
}
