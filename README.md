# ESP32 Indoor Positioning to Firebase Integration

A streamlined, single-script solution to ingest real-time telemetry from an ESP32 asset tracker, compute indoor coordinates $(X, Y)$ using an Adaptive Kalman Filter, and stream live positioning data directly to **Firebase Realtime Database**.

---

## 📁 Repository Structure

```
esp32_indoor_positioning/
│
├── esp32_firebase_tracker.py   # ⭐ Single self-contained Python script
├── firebase_config.json        # Configuration (Firebase URL, anchors, serial port)
├── requirements.txt            # Python dependencies (numpy, pyserial)
├── firmware/                   # Arduino sketches for ESP32 devices
│   ├── anchor_a/anchor_a.ino   # Fixed Access Point Anchor A (Channel 1)
│   ├── anchor_b/anchor_b.ino   # Fixed Access Point Anchor B (Channel 6)
│   └── tracker_c/tracker_c.ino # Moving Asset Tracker (MPU6050 + Wi-Fi scan)
│
├── archive/                    # Archived previous simulation, tests, & web UI
└── archieve/                   # Directory junction pointing to archive/
```

---

## 🚀 Quick Start

### 1. Install Dependencies
```bash
pip install -r requirements.txt
```

### 2. Configure Firebase & Hardware
Open [`firebase_config.json`](firebase_config.json) and set your Firebase Realtime Database URL:
```json
{
  "firebase": {
    "database_url": "https://<YOUR-PROJECT-ID>-default-rtdb.firebaseio.com",
    "auth_secret": "",
    "device_id": "tracker_01",
    "live_path": "current_location",
    "history_path": "location_history",
    "enable_history_logging": true,
    "send_rate_hz": 2.0
  },
  "serial": {
    "port": "AUTO",
    "baud_rate": 115200,
    "simulation_fallback": true
  }
}
```

* **Firebase Database URL**: Found in your Firebase Console under *Build > Realtime Database*.
* **Database Rules**: For development, ensure read/write rules allow writing:
  ```json
  {
    "rules": {
      ".read": true,
      ".write": true
    }
  }
  ```
  *(Or set `"auth_secret"` to your Database Secret under Project Settings > Service Accounts > Database secrets).*

### 3. Flash Hardware
1. **Anchor A**: Flash [`firmware/anchor_a/anchor_a.ino`](firmware/anchor_a/anchor_a.ino) to ESP32 #1 (placed at `[0.0, 0.0]`).
2. **Anchor B**: Flash [`firmware/anchor_b/anchor_b.ino`](firmware/anchor_b/anchor_b.ino) to ESP32 #2 (placed at `[10.0, 0.0]`).
3. **Tracker C**: Wire MPU6050 to ESP32 #3 (SDA: `GPIO 32`, SCL: `GPIO 33`, VCC: `3.3V`, GND: `GND`) and flash [`firmware/tracker_c/tracker_c.ino`](firmware/tracker_c/tracker_c.ino).

### 4. Run the Ingestion Engine
```bash
python esp32_firebase_tracker.py
```

Or pass parameters directly via CLI:
```bash
python esp32_firebase_tracker.py --port COM3 --firebase-url https://my-proj-default-rtdb.firebaseio.com
```

---

## 📡 Firebase Data Schema

The script pushes real-time positioning updates to two paths:

### 1. Live Snapshot: `/current_location/{device_id}`
Overwritten at 2 Hz (`PUT /current_location/tracker_01.json`):
```json
{
  "device_id": "tracker_01",
  "location": {
    "x": 3.42,
    "y": 1.15,
    "vx": 0.12,
    "vy": 0.04,
    "speed_mps": 0.13,
    "heading_deg": 48.2,
    "uncertainty_m": 0.42
  },
  "status": "TRACKING_ACTIVE",
  "anchor_distances": {
    "A": 3.61,
    "B": 6.68
  },
  "raw_rssi": {
    "A": -52,
    "B": -63
  },
  "timestamp_ms": 1730125430120,
  "timestamp_iso": "2026-10-28T14:23:50.120Z"
}
```

### 2. Historical Trail: `/location_history/{device_id}`
Appended with unique push IDs (`POST /location_history/tracker_01.json`) for path playback and analytics.
