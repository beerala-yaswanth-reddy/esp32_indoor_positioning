/*
 * ESP32 Indoor Positioning System - Device C (Moving Tracker)
 *
 * Hardware:
 *   - ESP32-WROVER Development Board (30-pin)
 *   - MPU6050 Accelerometer & Gyroscope (I2C address: 0x68)
 *
 * Pin Connections:
 *   - SDA: GPIO 32
 *   - SCL: GPIO 33
 *   - VCC: 3.3V
 *   - GND: Common Ground
 *   - AD0: Unconnected (defaults to address 0x68)
 *
 * Protocol: Newline-delimited JSON over USB Serial (115200 baud)
 * Non-blocking asynchronous Wi-Fi scanning prevents blocking the IMU sampling loop.
 */

#include <WiFi.h>
#include <Wire.h>

#define I2C_SDA_PIN 32
#define I2C_SCL_PIN 33
#define MPU6050_ADDR 0x68

// Target Anchor SSIDs
const char* ANCHOR_A_SSID = "INDOOR_ANCHOR_A";
const char* ANCHOR_B_SSID = "INDOOR_ANCHOR_B";
const char* DEVICE_ID = "tracker_01";

// Sentinel value for missing anchor measurements
const int SENTINEL_RSSI_MISSING = -127;

// Conversion factors for MPU6050
// Accel range +/- 2g: 16384 LSB/g, 1g = 9.80665 m/s^2
const float ACCEL_SCALE = 9.80665f / 16384.0f;
// Gyro range +/- 250 deg/s: 131 LSB / (deg/s), convert to rad/s
const float GYRO_SCALE = (3.14159265f / 180.0f) / 131.0f;

// Cached RSSI readings and timestamps
int rssi_A = SENTINEL_RSSI_MISSING;
int rssi_B = SENTINEL_RSSI_MISSING;
unsigned long rssi_A_time = 0;
unsigned long rssi_B_time = 0;
bool mpu_detected = false;

// Loop timing
unsigned long lastTelemetryTime = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 100; // 10 Hz telemetry update rate

// Non-blocking Wi-Fi scan state
bool scanInProgress = false;

void setupMPU6050() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000); // 400 kHz fast I2C
  
  Wire.beginTransmission(MPU6050_ADDR);
  byte error = Wire.endTransmission();
  
  if (error == 0) {
    mpu_detected = true;
    // Wake up MPU6050 (clearing SLEEP bit in PWR_MGMT_1 register 0x6B)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission(true);

    // Set accelerometer configuration (+/- 2g) in ACCEL_CONFIG (0x1C)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1C);
    Wire.write(0x00);
    Wire.endTransmission(true);

    // Set gyroscope configuration (+/- 250 deg/s) in GYRO_CONFIG (0x1B)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1B);
    Wire.write(0x00);
    Wire.endTransmission(true);
  } else {
    mpu_detected = false;
  }
}

void readMPU6050(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  if (!mpu_detected) {
    ax = ay = gx = gy = gz = 0.0f;
    az = 9.80665f;
    return;
  }

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B); // Starting at register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);

  if (Wire.available() >= 14) {
    int16_t raw_ax = (Wire.read() << 8) | Wire.read();
    int16_t raw_ay = (Wire.read() << 8) | Wire.read();
    int16_t raw_az = (Wire.read() << 8) | Wire.read();
    int16_t raw_temp = (Wire.read() << 8) | Wire.read();
    int16_t raw_gx = (Wire.read() << 8) | Wire.read();
    int16_t raw_gy = (Wire.read() << 8) | Wire.read();
    int16_t raw_gz = (Wire.read() << 8) | Wire.read();

    ax = raw_ax * ACCEL_SCALE;
    ay = raw_ay * ACCEL_SCALE;
    az = raw_az * ACCEL_SCALE;

    gx = raw_gx * GYRO_SCALE;
    gy = raw_gy * GYRO_SCALE;
    gz = raw_gz * GYRO_SCALE;
  }
}

void checkWifiScan() {
  if (!scanInProgress) {
    // Initiate non-blocking asynchronous scan
    // Parameters: async = true, show_hidden = false, passive = false, max_ms_per_chan = 120
    WiFi.scanNetworks(true, false, false, 120);
    scanInProgress = true;
  } else {
    int n = WiFi.scanComplete();
    if (n >= 0) {
      // Scan complete! Process results
      unsigned long now = millis();
      int found_a = SENTINEL_RSSI_MISSING;
      int found_b = SENTINEL_RSSI_MISSING;

      for (int i = 0; i < n; ++i) {
        String ssid = WiFi.SSID(i);
        int rssi = WiFi.RSSI(i);

        if (ssid.equals(ANCHOR_A_SSID)) {
          found_a = rssi;
        } else if (ssid.equals(ANCHOR_B_SSID)) {
          found_b = rssi;
        }
      }

      rssi_A = found_a;
      rssi_B = found_b;
      if (found_a != SENTINEL_RSSI_MISSING) rssi_A_time = now;
      if (found_b != SENTINEL_RSSI_MISSING) rssi_B_time = now;

      WiFi.scanDelete(); // Clean up memory
      scanInProgress = false;
    } else if (n == -1) {
      // Scan is still running; do not block!
    } else if (n == -2) {
      // Scan failed; reset
      scanInProgress = false;
    }
  }
}

void sendTelemetryJSON(float ax, float ay, float az, float gx, float gy, float gz) {
  unsigned long now = millis();
  unsigned long age_a = (rssi_A != SENTINEL_RSSI_MISSING) ? (now - rssi_A_time) : 9999;
  unsigned long age_b = (rssi_B != SENTINEL_RSSI_MISSING) ? (now - rssi_B_time) : 9999;

  // Format compact newline-delimited JSON
  Serial.printf(
    "{\"schema_version\":1,\"type\":\"telemetry\",\"device_id\":\"%s\",\"timestamp_ms\":%lu,"
    "\"imu\":{\"accel_mps2\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
    "\"gyro_rads\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},\"valid\":%s},"
    "\"anchors\":["
    "{\"anchor_id\":\"A\",\"ssid\":\"%s\",\"rssi_dbm\":%d,\"valid\":%s,\"age_ms\":%lu},"
    "{\"anchor_id\":\"B\",\"ssid\":\"%s\",\"rssi_dbm\":%d,\"valid\":%s,\"age_ms\":%lu}"
    "],\"status\":{\"sensor_ok\":%s,\"free_heap\":%u}}\n",
    DEVICE_ID,
    now,
    ax, ay, az,
    gx, gy, gz,
    mpu_detected ? "true" : "false",
    ANCHOR_A_SSID,
    rssi_A,
    (rssi_A != SENTINEL_RSSI_MISSING) ? "true" : "false",
    age_a,
    ANCHOR_B_SSID,
    rssi_B,
    (rssi_B != SENTINEL_RSSI_MISSING) ? "true" : "false",
    age_b,
    mpu_detected ? "true" : "false",
    ESP.getFreeHeap()
  );
}

void setup() {
  Serial.begin(115200);
  delay(300);

  // Initialize Station Mode for Wi-Fi scanning (do not auto-connect)
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Initialize MPU6050 on GPIO 32 (SDA) and GPIO 33 (SCL)
  setupMPU6050();

  delay(100);
}

void loop() {
  // 1. Asynchronous Wi-Fi scan state machine
  checkWifiScan();

  // 2. High-rate periodic sensor reading & JSON dispatch (10 Hz)
  unsigned long now = millis();
  if (now - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryTime = now;

    float ax, ay, az, gx, gy, gz;
    readMPU6050(ax, ay, az, gx, gy, gz);

    sendTelemetryJSON(ax, ay, az, gx, gy, gz);
  }

  // Small yield to FreeRTOS watchdog
  delay(5);
}
