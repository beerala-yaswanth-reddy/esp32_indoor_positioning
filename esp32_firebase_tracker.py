"""
ESP32 Indoor Positioning to Firebase Database Ingestion Engine
================================================================
A self-contained script that:
1. Connects to the ESP32 Tracker over USB Serial (115200 baud).
2. Parses 10 Hz NDJSON telemetry (IMU + Wi-Fi RSSI from Anchors A & B).
3. Computes 2D/1D real-time position using an Adaptive Kalman Filter,
   path-loss distance estimation, and Zero Velocity Update (ZUPT).
4. Employs Acuity-based Motion Detection (Acceleration Variance) and
   Position Deadband Clamping so that when the tracker is idle, it stays
   rock-solid at rest (0.0 m/s, status: STATIONARY).
5. Streams the live location and historical trajectory to Firebase Database
   (Firebase Realtime Database REST API or Firebase Firestore).
"""

import argparse
import datetime
import json
import math
import os
import sys
import threading
import time
import urllib.error
import urllib.request
from collections import deque
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


# ==============================================================================
# DEFAULT SYSTEM CONFIGURATION
# ==============================================================================
DEFAULT_CONFIG = {
    "serial": {
        "port": "AUTO",          # 'AUTO' to detect ESP32 port, or e.g. 'COM3' / '/dev/ttyUSB0'
        "baud_rate": 115200,
        "timeout_sec": 1.0,
        "reconnect_interval_sec": 2.0,
        "simulation_fallback": True  # Generates realistic movement if ESP32 not connected
    },
    "firebase": {
        "database_url": "",      # e.g., "https://your-project-default-rtdb.firebaseio.com"
        "auth_secret": "",       # Optional Database Secret or Auth ID Token
        "device_id": "tracker_01",
        "live_path": "current_location",    # Writes to /current_location/{device_id}.json
        "history_path": "location_history", # Appends to /location_history/{device_id}.json
        "enable_history_logging": True,
        "send_rate_hz": 2.0                 # Rate to sync with Firebase (Hz)
    },
    "anchors": [
        {"id": "A", "ssid": "INDOOR_ANCHOR_A", "coords": [0.0, 0.0]},
        {"id": "B", "ssid": "INDOOR_ANCHOR_B", "coords": [10.0, 0.0]}
    ],
    "estimator": {
        "d0_meters": 1.0,
        "rssi_0_dbm": -45.0,
        "path_loss_exponent": 2.4,
        "stale_timeout_sec": 2.5,
        "dt": 0.1,
        "motion_threshold_mps2": 0.08,   # Std-dev of acceleration magnitude indicating movement
        "position_deadband_m": 0.65,     # Deadband radius to prevent RF jitter drift while stationary
        "kalman": {
            "q_pos": 0.02,
            "q_vel": 0.15,
            "r_base": 3.0,
            "adaptive_spread_threshold": 4.5,
            "high_noise_multiplier": 5.0
        }
    }
}


# ==============================================================================
# NOISE FILTERING & POSITION ESTIMATION
# ==============================================================================
class RSSIFilter:
    """Sliding-window median + EMA filter with step limiter and micro-deadband."""

    def __init__(self, window_size: int = 7, ema_alpha: float = 0.25):
        self.history: deque = deque(maxlen=window_size)
        self.alpha = ema_alpha
        self.ema_value: Optional[float] = None
        self.last_update_time: float = 0.0

    def update(self, raw_rssi: float, current_time: float) -> float:
        # Rejection of abrupt multipath spikes (> 8 dBm in a single 100ms frame)
        if self.ema_value is not None:
            delta = raw_rssi - self.ema_value
            if abs(delta) > 8.0:
                raw_rssi = self.ema_value + math.copysign(8.0, delta)

        self.history.append(raw_rssi)
        self.last_update_time = current_time

        # Step 1: Median over sliding window to reject RF multipath glitches
        median_val = float(np.median(list(self.history)))

        # Step 2: Exponential moving average with deadband to avoid endless micro-jitter
        if self.ema_value is None:
            self.ema_value = median_val
        else:
            if abs(median_val - self.ema_value) >= 0.7:  # Deadband: ignore micro-noise < 0.7 dBm
                self.ema_value = self.alpha * median_val + (1.0 - self.alpha) * self.ema_value

        return self.ema_value

    def is_stale(self, current_time: float, timeout_sec: float) -> bool:
        return (current_time - self.last_update_time) > timeout_sec


class AdaptiveKalmanPositionEstimator:
    """
    Self-contained 4-State Adaptive Kalman Filter fusing Wi-Fi RSSI
    trilateration with MPU6050 accelerometer & Zero Velocity Updates (ZUPT).
    Includes Acuity-based Motion Detection and Stationary Position Clamping.
    State: [x, y, vx, vy]
    """

    def __init__(self, cfg: dict):
        self.cfg = cfg
        est_cfg = cfg.get("estimator", {})
        self.d0 = float(est_cfg.get("d0_meters", 1.0))
        self.rssi_0 = float(est_cfg.get("rssi_0_dbm", -45.0))
        self.ple = float(est_cfg.get("path_loss_exponent", 2.4))
        self.stale_timeout = float(est_cfg.get("stale_timeout_sec", 2.5))
        self.dt = float(est_cfg.get("dt", 0.1))
        self.motion_threshold = float(est_cfg.get("motion_threshold_mps2", 0.08))
        self.position_deadband_m = float(est_cfg.get("position_deadband_m", 0.65))

        # Anchor coordinates
        self.anchors = {
            a["id"].upper(): np.array(a["coords"], dtype=float)
            for a in cfg.get("anchors", [])
        }
        self.rssi_filters = {aid: RSSIFilter() for aid in self.anchors}

        # Kalman Filter Matrices
        dt = self.dt
        self.F = np.array([
            [1.0, 0.0, dt, 0.0],
            [0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ], dtype=float)

        self.B = np.array([
            [0.5 * dt**2, 0.0],
            [0.0, 0.5 * dt**2],
            [dt, 0.0],
            [0.0, dt]
        ], dtype=float)

        self.H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0]
        ], dtype=float)

        kf_cfg = est_cfg.get("kalman", {})
        q_pos = float(kf_cfg.get("q_pos", 0.02))
        q_vel = float(kf_cfg.get("q_vel", 0.15))
        self.Q = np.diag([q_pos, q_pos, q_vel, q_vel])

        self.r_base = float(kf_cfg.get("r_base", 3.0))
        self.R_nominal = np.diag([self.r_base, self.r_base])
        self.spread_threshold = float(kf_cfg.get("adaptive_spread_threshold", 4.5))
        self.high_noise_mult = float(kf_cfg.get("high_noise_multiplier", 5.0))

        # Internal Filter States
        self.state = np.array([0.0, 0.0, 0.0, 0.0], dtype=float)
        self.P = np.diag([1.0, 1.0, 0.5, 0.5])
        self.heading_rad: float = 0.0
        self.has_first_fix: bool = False
        self.last_predict_time: float = time.time()

        # Dynamic Startup IMU Bias Calibration
        self.calibrating = True
        self.calib_samples_target = 30
        self.calib_buffer_ax: List[float] = []
        self.calib_buffer_ay: List[float] = []
        self.calib_buffer_gz: List[float] = []
        self.bias_ax = 0.0
        self.bias_ay = 0.0
        self.bias_gz = 0.0

        # Motion & Stationary State Detection
        self.accel_history: deque = deque(maxlen=15)
        self.is_stationary = True
        self.stationary_streak = 5
        self.moving_streak = 0
        self.locked_position: Optional[Tuple[float, float]] = None
        self.accel_variance = 0.0

    def rssi_to_distance(self, rssi_dbm: float) -> float:
        """Log-distance path-loss model: d = d0 * 10 ^ ((RSSI_0 - RSSI) / (10 * PLE))"""
        exponent = (self.rssi_0 - rssi_dbm) / (10.0 * self.ple)
        exponent = max(-2.0, min(exponent, 4.0))  # numerical stability clamp
        return max(0.1, float(self.d0 * (10.0 ** exponent)))

    def solve_geometric_position(self, distances: Dict[str, float]) -> Optional[Tuple[float, float]]:
        """Compute geometric (x, y) from anchor ranges."""
        valid_anchors = [aid for aid in distances if aid in self.anchors]
        if len(valid_anchors) < 2:
            return None

        if len(valid_anchors) == 2 and "A" in distances and "B" in distances:
            # 2-Anchor Corridor Trilateration along baseline
            pA = self.anchors["A"]
            pB = self.anchors["B"]
            dA = distances["A"]
            dB = distances["B"]
            dAB = float(np.linalg.norm(pB - pA))
            if dAB < 1e-4:
                return float(pA[0]), float(pA[1])

            # 1D projection along baseline
            x_proj = (dA**2 - dB**2 + dAB**2) / (2.0 * dAB)
            x_proj = max(0.0, min(x_proj, dAB))

            # Residual for lateral corridor displacement
            y_sq = max(0.0, dA**2 - x_proj**2)
            y_cand = math.sqrt(y_sq)
            # Corridor disambiguation: choose lateral displacement nearest to current state
            y_proj = y_cand if abs(y_cand - self.state[1]) <= abs(-y_cand - self.state[1]) else -y_cand
            return float(x_proj), float(y_proj)

        elif len(valid_anchors) >= 3:
            # Multilateration linear least squares
            ref_id = valid_anchors[0]
            xr, yr = self.anchors[ref_id]
            rr = distances[ref_id]

            A_rows, b_rows = [], []
            for aid in valid_anchors[1:]:
                xi, yi = self.anchors[aid]
                ri = distances[aid]
                A_rows.append([2.0 * (xi - xr), 2.0 * (yi - yr)])
                b_rows.append(rr**2 - ri**2 + xi**2 + yi**2 - xr**2 - yr**2)

            A_mat = np.array(A_rows, dtype=float)
            b_vec = np.array(b_rows, dtype=float)
            try:
                coords, _, _, _ = np.linalg.lstsq(A_mat, b_vec, rcond=None)
                return float(coords[0]), float(coords[1])
            except np.linalg.LinAlgError:
                return None

        return None

    def update(self, telemetry: dict) -> dict:
        """Process incoming NDJSON telemetry message and update estimated position."""
        now = time.time()
        dt = max(0.01, min(now - self.last_predict_time, 1.0))
        self.last_predict_time = now

        # 1. Parse IMU & Perform Calibration
        imu = telemetry.get("imu", {})
        accel = imu.get("accel_mps2", {})
        gyro = imu.get("gyro_rads", {})

        raw_ax = float(accel.get("x", 0.0))
        raw_ay = float(accel.get("y", 0.0))
        raw_gz = float(gyro.get("z", 0.0))

        # Handle automatic startup calibration
        if self.calibrating:
            self.calib_buffer_ax.append(raw_ax)
            self.calib_buffer_ay.append(raw_ay)
            self.calib_buffer_gz.append(raw_gz)
            if len(self.calib_buffer_ax) >= self.calib_samples_target:
                self.bias_ax = float(np.mean(self.calib_buffer_ax))
                self.bias_ay = float(np.mean(self.calib_buffer_ay))
                self.bias_gz = float(np.mean(self.calib_buffer_gz))
                self.calibrating = False

        # Subtract baseline zero-bias offsets
        ax = raw_ax - self.bias_ax
        ay = raw_ay - self.bias_ay
        gz = raw_gz - self.bias_gz

        # 2. Acuity-Based Motion Detection (Variance of Acceleration Magnitude)
        accel_mag = math.sqrt(ax**2 + ay**2)
        self.accel_history.append(accel_mag)

        if len(self.accel_history) >= 8:
            self.accel_variance = float(np.std(list(self.accel_history)))
            accel_mean = float(np.mean(list(self.accel_history)))
        else:
            self.accel_variance = 0.0
            accel_mean = accel_mag

        # Stationary condition: acceleration variance is minimal (resting on surface)
        instant_stationary = (self.accel_variance < self.motion_threshold) and (accel_mean < 0.25)
        if instant_stationary:
            self.stationary_streak += 1
            self.moving_streak = 0
            if self.stationary_streak >= 4:
                self.is_stationary = True
        else:
            self.moving_streak += 1
            if self.moving_streak >= 3:
                self.is_stationary = False
                self.stationary_streak = 0

        # Gyroscope integration (suppress heading drift when stationary)
        if self.is_stationary or abs(gz) < 0.02:
            gz = 0.0
        self.heading_rad = (self.heading_rad + gz * dt) % (2.0 * math.pi)

        # 3. Filter Wi-Fi RSSI & calculate distances
        distances = {}
        raw_rssis = []
        for a_info in telemetry.get("anchors", []):
            aid = str(a_info.get("anchor_id", "")).upper()
            rssi = float(a_info.get("rssi_dbm", -127))
            valid = bool(a_info.get("valid", False)) and (rssi > -100.0)

            if valid and aid in self.rssi_filters:
                filt_rssi = self.rssi_filters[aid].update(rssi, now)
                raw_rssis.append(filt_rssi)
                distances[aid] = self.rssi_to_distance(filt_rssi)

        # 4. Kalman Prediction Step
        if self.is_stationary:
            # Force velocity to 0 and remove acceleration input when stationary
            self.state[2] = 0.0
            self.state[3] = 0.0
            u = np.zeros(2, dtype=float)
            # Continuous gentle background bias trimming
            self.bias_ax = 0.995 * self.bias_ax + 0.005 * raw_ax
            self.bias_ay = 0.995 * self.bias_ay + 0.005 * raw_ay
            self.bias_gz = 0.995 * self.bias_gz + 0.005 * raw_gz
        else:
            u = np.array([ax, ay], dtype=float)

        x_pred = self.F @ self.state + self.B @ u
        P_pred = self.F @ self.P @ self.F.T + self.Q

        # 5. Kalman Measurement Update Step
        geom_pos = self.solve_geometric_position(distances)
        measurement_valid = False

        if geom_pos is not None:
            z = np.array(geom_pos, dtype=float)

            if not self.has_first_fix:
                self.state[0] = z[0]
                self.state[1] = z[1]
                self.locked_position = (float(z[0]), float(z[1]))
                self.has_first_fix = True
                x_pred[:2] = z

            # Deadband Clamping: If stationary, lock position against RF noise
            if self.is_stationary and self.locked_position is not None:
                dist_from_locked = math.sqrt(
                    (z[0] - self.locked_position[0])**2 +
                    (z[1] - self.locked_position[1])**2
                )
                if dist_from_locked < self.position_deadband_m:
                    # Within deadband: keep position locked to resting coordinates
                    z = np.array(self.locked_position, dtype=float)
                else:
                    # Sustained displacement: gently adapt locked position
                    self.locked_position = (
                        0.9 * self.locked_position[0] + 0.1 * z[0],
                        0.9 * self.locked_position[1] + 0.1 * z[1]
                    )
            else:
                self.locked_position = (float(z[0]), float(z[1]))

            # Adaptive Measurement Noise Scaling
            R = self.R_nominal.copy()
            if self.is_stationary:
                R *= 4.0  # Strongly de-prioritize noisy Wi-Fi fluctuations while stationary

            if len(raw_rssis) >= 2:
                spread = float(np.std(raw_rssis))
                if spread > self.spread_threshold:
                    R *= self.high_noise_mult

            # Kalman Gain
            y_res = z - (self.H @ x_pred)
            S = self.H @ P_pred @ self.H.T + R
            K = P_pred @ self.H.T @ np.linalg.inv(S)

            self.state = x_pred + (K @ y_res)
            self.P = (np.eye(4) - K @ self.H) @ P_pred
            measurement_valid = True
        else:
            self.state = x_pred
            self.P = P_pred

        # Enforce strict stationary clamping
        if self.is_stationary:
            self.state[2] = 0.0
            self.state[3] = 0.0
            if self.locked_position is not None:
                self.state[0] = self.locked_position[0]
                self.state[1] = self.locked_position[1]

        # 6. Build Result Payload
        pos_x = float(self.state[0])
        pos_y = float(self.state[1])
        vx = 0.0 if self.is_stationary else float(self.state[2])
        vy = 0.0 if self.is_stationary else float(self.state[3])
        speed = 0.0 if self.is_stationary else float(math.sqrt(vx**2 + vy**2))
        heading_deg = float(math.degrees(self.heading_rad) % 360.0)
        uncertainty = float(math.sqrt(max(0.01, self.P[0, 0] + self.P[1, 1])))

        status_str = "STATIONARY" if self.is_stationary else "MOVING"

        return {
            "x": round(pos_x, 3),
            "y": round(pos_y, 3),
            "vx": round(vx, 3),
            "vy": round(vy, 3),
            "speed_mps": round(speed, 2),
            "heading_deg": round(heading_deg, 1),
            "uncertainty_m": round(uncertainty, 2),
            "status": status_str,
            "is_stationary": self.is_stationary,
            "accel_variance": round(self.accel_variance, 4),
            "anchor_distances": {k: round(v, 2) for k, v in distances.items()},
            "raw_rssi": {
                a.get("anchor_id"): a.get("rssi_dbm")
                for a in telemetry.get("anchors", [])
            }
        }


# ==============================================================================
# FIREBASE DATABASE DISPATCH CLIENT
# ==============================================================================
class FirebaseClient:
    """
    Robust Firebase Realtime Database Client using Python's standard HTTP library.
    Works seamlessly with Firebase Database REST endpoints.
    """

    def __init__(self, db_url: str, auth_secret: str = ""):
        self.db_url = db_url.rstrip("/") if db_url else ""
        self.auth_secret = auth_secret.strip()
        self.is_configured = bool(self.db_url and not self.db_url.startswith("https://your-project"))
        self.lock = threading.Lock()
        self.last_status: str = "Ready" if self.is_configured else "Unconfigured (Dry Run)"

    def _build_url(self, path: str) -> str:
        clean_path = path.strip("/").lstrip("/")
        url = f"{self.db_url}/{clean_path}.json"
        if self.auth_secret:
            url += f"?auth={self.auth_secret}"
        return url

    def set_data(self, path: str, payload: dict) -> bool:
        """PUT /path.json - Overwrites or updates live record."""
        if not self.is_configured:
            return False
        url = self._build_url(path)
        data_bytes = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(url, data=data_bytes, method="PUT")
        req.add_header("Content-Type", "application/json")

        try:
            with urllib.request.urlopen(req, timeout=4.0) as resp:
                success = 200 <= resp.status < 300
                self.last_status = f"OK ({resp.status})"
                return success
        except urllib.error.HTTPError as e:
            self.last_status = f"HTTP Error {e.code}: {e.reason}"
            return False
        except Exception as e:
            self.last_status = f"Network Error: {str(e)}"
            return False

    def push_data(self, path: str, payload: dict) -> bool:
        """POST /path.json - Appends historical log entry with unique ID."""
        if not self.is_configured:
            return False
        url = self._build_url(path)
        data_bytes = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(url, data=data_bytes, method="POST")
        req.add_header("Content-Type", "application/json")

        try:
            with urllib.request.urlopen(req, timeout=4.0) as resp:
                success = 200 <= resp.status < 300
                return success
        except Exception:
            return False


# ==============================================================================
# SERIAL COMMUNICATION & SIMULATION FALLBACK
# ==============================================================================
class ESP32SerialReader:
    """Reads NDJSON packets from physical ESP32 or provides simulated movement."""

    def __init__(self, port: str = "AUTO", baud_rate: int = 115200, simulation_fallback: bool = True):
        self.requested_port = port
        self.baud_rate = baud_rate
        self.simulation_fallback = simulation_fallback
        self.serial_handle: Optional[Any] = None
        self.is_simulating = False
        self.active_port_name = "None"
        self._sim_t = 0.0

    def find_esp32_port(self) -> Optional[str]:
        if not SERIAL_AVAILABLE:
            return None
        ports = list(serial.tools.list_ports.comports())
        if not ports:
            return None

        # Prioritize USB-UART chips common on ESP32 development boards
        identifiers = ["CP210", "CH340", "CH9102", "FTDI", "UART", "ESP32", "USB-to-Serial"]
        for p in ports:
            desc = f"{p.description} {p.manufacturer} {p.hwid}".upper()
            if any(ident in desc for ident in identifiers):
                return p.device

        # Fallback to the first available COM port
        return ports[0].device

    def connect(self) -> bool:
        if not SERIAL_AVAILABLE:
            if self.simulation_fallback:
                self.is_simulating = True
                self.active_port_name = "SIMULATOR"
                return True
            return False

        target_port = self.requested_port
        if target_port == "AUTO":
            detected = self.find_esp32_port()
            if not detected:
                if self.simulation_fallback:
                    self.is_simulating = True
                    self.active_port_name = "SIMULATOR (Fallback)"
                    return True
                return False
            target_port = detected

        try:
            self.serial_handle = serial.Serial(
                port=target_port,
                baudrate=self.baud_rate,
                timeout=0.5
            )
            self.active_port_name = target_port
            self.is_simulating = False
            time.sleep(0.5)  # Let ESP32 finish reset upon DTR/RTS trigger
            return True
        except Exception as e:
            if self.simulation_fallback:
                self.is_simulating = True
                self.active_port_name = "SIMULATOR (Port Error)"
                return True
            print(f"[!] Could not open serial port {target_port}: {e}")
            return False

    def read_telemetry(self) -> Optional[dict]:
        """Returns parsed telemetry dict or None."""
        if self.is_simulating:
            time.sleep(0.1)
            self._sim_t += 0.1
            true_x = 5.0 + 3.5 * math.cos(0.5 * self._sim_t)
            true_y = 1.0 * math.sin(0.5 * self._sim_t)

            dA = max(0.2, math.sqrt((true_x - 0.0)**2 + (true_y - 0.0)**2))
            dB = max(0.2, math.sqrt((true_x - 10.0)**2 + (true_y - 0.0)**2))
            rssi_a = int(-45.0 - 10.0 * 2.4 * math.log10(dA / 1.0) + np.random.normal(0, 1.5))
            rssi_b = int(-45.0 - 10.0 * 2.4 * math.log10(dB / 1.0) + np.random.normal(0, 1.5))

            return {
                "schema_version": 1,
                "type": "telemetry",
                "device_id": "sim_tracker",
                "timestamp_ms": int(time.time() * 1000),
                "imu": {
                    "accel_mps2": {"x": 0.05, "y": 0.0, "z": 9.81},
                    "gyro_rads": {"x": 0.0, "y": 0.0, "z": 0.01},
                    "valid": True
                },
                "anchors": [
                    {"anchor_id": "A", "ssid": "INDOOR_ANCHOR_A", "rssi_dbm": rssi_a, "valid": True},
                    {"anchor_id": "B", "ssid": "INDOOR_ANCHOR_B", "rssi_dbm": rssi_b, "valid": True}
                ]
            }

        if not self.serial_handle or not self.serial_handle.is_open:
            return None

        try:
            line = self.serial_handle.readline().decode("utf-8", errors="ignore").strip()
            if not line or not line.startswith("{"):
                return None
            return json.loads(line)
        except Exception:
            return None

    def close(self):
        if self.serial_handle and self.serial_handle.is_open:
            self.serial_handle.close()


# ==============================================================================
# MAIN SYSTEM CONTROLLER & ORCHESTRATION
# ==============================================================================
def main():
    parser = argparse.ArgumentParser(
        description="ESP32 Indoor Positioning to Firebase Database Engine"
    )
    parser.add_argument(
        "--config", type=str, default="firebase_config.json",
        help="Path to JSON configuration file (default: firebase_config.json)"
    )
    parser.add_argument("--port", type=str, default=None, help="Serial COM port (e.g. COM3, AUTO)")
    parser.add_argument("--baud", type=int, default=None, help="Serial baud rate (default: 115200)")
    parser.add_argument("--firebase-url", type=str, default=None, help="Firebase Database URL")
    parser.add_argument("--firebase-secret", type=str, default=None, help="Firebase Database Secret")
    parser.add_argument("--no-sim", action="store_true", help="Disable simulated fallback")
    args = parser.parse_args()

    # Load configuration
    config = DEFAULT_CONFIG.copy()
    if os.path.exists(args.config):
        try:
            with open(args.config, "r", encoding="utf-8") as f:
                loaded = json.load(f)
                config.update(loaded)
                print(f"[*] Loaded settings from: {args.config}")
        except Exception as e:
            print(f"[!] Warning reading '{args.config}': {e}. Using defaults.")

    # Override with CLI arguments if provided
    if args.port:
        config["serial"]["port"] = args.port
    if args.baud:
        config["serial"]["baud_rate"] = args.baud
    if args.firebase_url:
        config["firebase"]["database_url"] = args.firebase_url
    if args.firebase_secret:
        config["firebase"]["auth_secret"] = args.firebase_secret
    if args.no_sim:
        config["serial"]["simulation_fallback"] = False

    device_id = config.get("firebase", {}).get("device_id", "tracker_01")
    fb_url = config.get("firebase", {}).get("database_url", "")
    fb_secret = config.get("firebase", {}).get("auth_secret", "")
    send_interval = 1.0 / max(0.2, float(config.get("firebase", {}).get("send_rate_hz", 2.0)))

    print("=" * 70)
    print("      ESP32 INDOOR POSITIONING -> FIREBASE INTEGRATION ENGINE")
    print("=" * 70)
    print(f"[*] Target Device ID:   {device_id}")
    print(f"[*] Serial Port Config: {config['serial']['port']} @ {config['serial']['baud_rate']} baud")
    print(f"[*] Firebase RTDB URL:  {fb_url or '(None - Running in Console Preview Mode)'}")
    print(f"[*] Firebase Sync Rate: {round(1.0 / send_interval, 1)} Hz")
    print(f"[*] Anchors Configured: {len(config.get('anchors', []))} beacons")
    for a in config.get("anchors", []):
        print(f"    - Anchor {a['id']}: '{a.get('ssid')}' at {a.get('coords')}")
    print("=" * 70)

    if not fb_url or fb_url.startswith("https://your-project"):
        print("[TIP] To sync live to Firebase, set 'database_url' in 'firebase_config.json'")
        print("      or pass --firebase-url https://<PROJECT>-default-rtdb.firebaseio.com")
        print("-" * 70)

    # Initialize Components
    estimator = AdaptiveKalmanPositionEstimator(config)
    firebase = FirebaseClient(fb_url, fb_secret)
    serial_reader = ESP32SerialReader(
        port=config["serial"]["port"],
        baud_rate=int(config["serial"]["baud_rate"]),
        simulation_fallback=bool(config["serial"]["simulation_fallback"])
    )

    print("[*] Connecting to ESP32 Serial...")
    if not serial_reader.connect():
        print("[!] ERROR: Failed to connect to ESP32 and simulation fallback is disabled.")
        sys.exit(1)

    print(f"[+] Active Source: {serial_reader.active_port_name}")
    print("[+] Starting Live Tracking Loop. Press Ctrl+C to terminate.\n")

    last_firebase_send = 0.0
    packets_received = 0
    start_time = time.time()

    try:
        while True:
            telemetry = serial_reader.read_telemetry()
            if not telemetry:
                time.sleep(0.01)
                continue

            packets_received += 1
            # 1. Compute estimated location
            pos_data = estimator.update(telemetry)

            now_time = time.time()
            iso_now = datetime.datetime.now(datetime.timezone.utc).isoformat()

            # 2. Sync to Firebase at configured throttled rate
            if (now_time - last_firebase_send) >= send_interval:
                last_firebase_send = now_time

                firebase_payload = {
                    "device_id": device_id,
                    "location": {
                        "x": pos_data["x"],
                        "y": pos_data["y"],
                        "vx": pos_data["vx"],
                        "vy": pos_data["vy"],
                        "speed_mps": pos_data["speed_mps"],
                        "heading_deg": pos_data["heading_deg"],
                        "uncertainty_m": pos_data["uncertainty_m"]
                    },
                    "status": pos_data["status"],
                    "motion": {
                        "is_stationary": pos_data["is_stationary"],
                        "accel_variance": pos_data["accel_variance"]
                    },
                    "anchor_distances": pos_data["anchor_distances"],
                    "raw_rssi": pos_data["raw_rssi"],
                    "timestamp_ms": int(now_time * 1000),
                    "timestamp_iso": iso_now
                }

                # Push to live endpoint (/current_location/{device_id}.json)
                live_path = f"{config['firebase']['live_path']}/{device_id}"
                fb_ok = firebase.set_data(live_path, firebase_payload)

                # Append to history log (/location_history/{device_id}.json)
                if config["firebase"].get("enable_history_logging", True) and fb_ok:
                    hist_path = f"{config['firebase']['history_path']}/{device_id}"
                    firebase.push_data(hist_path, firebase_payload)

            # 3. Print Clean Status Line to Terminal
            rssi_summary = ", ".join([
                f"{aid}:{val}dBm" for aid, val in pos_data["raw_rssi"].items() if val is not None
            ]) or "Scanning..."
            fb_tag = f"FB: {firebase.last_status}"

            mode_tag = "[IDLE/STATIONARY]" if pos_data["is_stationary"] else "[MOVING]"

            sys.stdout.write(
                f"\r{mode_tag} Pos: (X={pos_data['x']:>5.2f}m, Y={pos_data['y']:>5.2f}m) | "
                f"V: {pos_data['speed_mps']:>4.2f}m/s | "
                f"Var: {pos_data['accel_variance']:>5.3f} | "
                f"RSSI: [{rssi_summary}] | "
                f"{fb_tag}    "
            )
            sys.stdout.flush()

    except KeyboardInterrupt:
        print("\n\n[*] Shutting down...")
    finally:
        serial_reader.close()
        uptime = round(time.time() - start_time, 1)
        print(f"[*] Processed {packets_received} telemetry frames over {uptime}s.")
        print("[*] Tracker exited cleanly.")


if __name__ == "__main__":
    main()
