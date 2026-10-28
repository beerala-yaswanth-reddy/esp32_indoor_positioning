"""
Position Estimation Engine for ESP32 Indoor Positioning.

Implements:
1. RSSI-to-distance conversion using calibrated log-distance path-loss model
2. Noise filtering (Median filter + Exponential Moving Average)
3. 2-Anchor geometric solvers:
   - 1D linear corridor/hallway projection along anchor baseline
   - 2D circle-circle intersection with prior/motion disambiguation
4. N-Anchor (>=3) multilateration (generalized linear least-squares from simulation)
5. 4-State Adaptive Kalman Filter fusing Wi-Fi ranges with MPU6050 IMU
6. Sensor bias tracking and Zero Velocity Update (ZUPT) to prevent acceleration drift
7. Explicit validity status, sentinel handling, and covariance-based uncertainty
"""

import math
import time
from collections import deque
from typing import Dict, List, Optional, Tuple, Any
import numpy as np

from core.protocol import TelemetryMessage, PositionEstimate, SENTINEL_RSSI_MISSING


class RSSIFilter:
    """Sliding-window median + EMA filter for raw RSSI samples."""

    def __init__(self, window_size: int = 5, ema_alpha: float = 0.35):
        self.window_size = window_size
        self.alpha = ema_alpha
        self.history: deque = deque(maxlen=window_size)
        self.ema_value: Optional[float] = None
        self.last_update_time: float = 0.0

    def update(self, raw_rssi: float, current_time: float) -> float:
        self.history.append(raw_rssi)
        self.last_update_time = current_time

        # Step 1: Median over sliding window to reject multipath spikes
        median_val = float(np.median(list(self.history)))

        # Step 2: Exponential moving average
        if self.ema_value is None:
            self.ema_value = median_val
        else:
            self.ema_value = self.alpha * median_val + (1.0 - self.alpha) * self.ema_value

        return self.ema_value

    def is_stale(self, current_time: float, timeout_sec: float) -> bool:
        return (current_time - self.last_update_time) > timeout_sec


class PositionEstimator:
    """
    Modular Indoor Position Estimator supporting 2 Wi-Fi anchors (extensible to N)
    fused with MPU6050 accelerometer & gyroscope measurements.
    """

    def __init__(self, config: Dict[str, Any]):
        self.config = config
        est_cfg = config.get("estimator", {})
        self.d0 = float(est_cfg.get("d0_meters", 1.0))
        self.rssi_0 = float(est_cfg.get("rssi_0_dbm", -45.0))
        self.ple = float(est_cfg.get("path_loss_exponent", 2.4))
        self.stale_timeout_sec = float(est_cfg.get("stale_timeout_ms", 2500)) / 1000.0
        self.mode = est_cfg.get("mode", "adaptive_kalman")
        self.line_constraint_y = float(est_cfg.get("line_constraint_y", 0.0))

        # Anchor database {id: {"ssid": str, "coords": np.ndarray, "enabled": bool}}
        self.anchors: Dict[str, Dict[str, Any]] = {}
        for a in config.get("anchors", []):
            a_id = a.get("id", "").upper()
            coords = np.array(a.get("coordinates", [0.0, 0.0]), dtype=float)
            self.anchors[a_id] = {
                "id": a_id,
                "ssid": a.get("ssid", f"INDOOR_ANCHOR_{a_id}"),
                "coords": coords,
                "enabled": a.get("enabled", True),
            }

        # Filter buffers per anchor
        window = int(est_cfg.get("median_window_size", 5))
        alpha = float(est_cfg.get("ema_alpha", 0.35))
        self.rssi_filters: Dict[str, RSSIFilter] = {
            a_id: RSSIFilter(window, alpha) for a_id in self.anchors
        }

        # Kalman Filter setup (State: [x, y, vx, vy])
        dt = float(est_cfg.get("dt", 0.1))
        self.dt = dt
        self.F = np.array([
            [1.0, 0.0, dt, 0.0],
            [0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ], dtype=float)

        self.B = np.array([
            [0.5 * dt**2, 0.0],
            [0.0, 0.5 * dt**2],
            [dt, 0.0],
            [0.0, dt],
        ], dtype=float)

        self.H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
        ], dtype=float)

        kf_cfg = est_cfg.get("kalman", {})
        q_pos = float(kf_cfg.get("q_pos", 0.02))
        q_vel = float(kf_cfg.get("q_vel", 0.15))
        self.Q = np.diag([q_pos, q_pos, q_vel, q_vel])

        self.r_base = float(kf_cfg.get("r_base", 3.0))
        self.R_nominal = np.diag([self.r_base, self.r_base])
        self.adaptive_scaling = bool(kf_cfg.get("adaptive_scaling", True))
        self.spread_threshold = float(kf_cfg.get("adaptive_spread_threshold", 4.5))
        self.high_noise_mult = float(kf_cfg.get("high_noise_multiplier", 5.0))

        # IMU calibration & ZUPT parameters
        imu_cfg = est_cfg.get("imu", {})
        self.accel_bias_x = float(imu_cfg.get("accel_bias_x", 0.0))
        self.accel_bias_y = float(imu_cfg.get("accel_bias_y", 0.0))
        self.zupt_threshold = float(imu_cfg.get("zupt_threshold_mps2", 0.25))

        # Internal Filter States
        self.state = np.array([0.0, 0.0, 0.0, 0.0], dtype=float)
        self.P = np.diag([1.0, 1.0, 0.5, 0.5])
        self.heading_rad: float = 0.0
        self.last_predict_time: float = time.time()
        self.has_first_fix: bool = False

    def rssi_to_distance(self, rssi_dbm: float) -> float:
        """
        Log-distance path-loss conversion:
        d = d0 * 10 ^ ((RSSI_0 - RSSI) / (10 * PLE))
        """
        exponent = (self.rssi_0 - rssi_dbm) / (10.0 * self.ple)
        # Protect against numerical overflow / underflow
        exponent = max(-2.0, min(exponent, 4.0))
        return self.d0 * math.pow(10.0, exponent)

    def solve_1d_line(self, d_a: float, d_b: float) -> Tuple[float, float, float]:
        """
        Solves 1D coordinate along the baseline between Anchor A and Anchor B.
        Anchor A is assumed at (0, y_line) and Anchor B at (D, y_line).
        Formula: x = (d_a^2 - d_b^2 + D^2) / (2 * D)
        Returns (x, y, geometric_residual)
        """
        pos_a = self.anchors.get("A", {}).get("coords", np.array([0.0, 0.0]))
        pos_b = self.anchors.get("B", {}).get("coords", np.array([10.0, 0.0]))
        D = float(np.linalg.norm(pos_b - pos_a))
        if D < 0.001:
            return 0.0, self.line_constraint_y, 999.0

        x_raw = (d_a**2 - d_b**2 + D**2) / (2.0 * D)
        # Soft clamping to avoid wildly absurd bounds while preserving dynamics
        x_clamped = max(-2.0, min(x_raw, D + 2.0))

        # Residual check against triangle inequality
        ideal_da = abs(x_clamped)
        ideal_db = abs(D - x_clamped)
        residual = abs(ideal_da - d_a) + abs(ideal_db - d_b)

        return float(x_clamped), self.line_constraint_y, float(residual)

    def solve_2d_circles(
        self, d_a: float, d_b: float, prior_y: float
    ) -> Tuple[float, float, float]:
        """
        Computes intersection of two circles centered at Anchor A and Anchor B.
        If circles intersect at 2 points, uses prior_y or IMU motion vector to disambiguate.
        If circles do not intersect (too far apart or one inside another), projects to the line of centers.
        """
        p_a = self.anchors.get("A", {}).get("coords", np.array([0.0, 0.0]))
        p_b = self.anchors.get("B", {}).get("coords", np.array([10.0, 0.0]))
        D = float(np.linalg.norm(p_b - p_a))
        if D < 0.001:
            return float(p_a[0]), float(p_a[1]), 999.0

        # Normal unit vector from A to B
        ex = (p_b - p_a) / D
        ey = np.array([-ex[1], ex[0]])

        # Chord distance along baseline
        d_diff = d_a**2 - d_b**2 + D**2
        x = d_diff / (2.0 * D)

        h_sq = d_a**2 - x**2
        if h_sq > 0.0:
            h = math.sqrt(h_sq)
            # Two solutions
            pt1 = p_a + x * ex + h * ey
            pt2 = p_a + x * ex - h * ey

            # Choose solution closest to prior_y
            if abs(pt1[1] - prior_y) <= abs(pt2[1] - prior_y):
                chosen = pt1
            else:
                chosen = pt2
            residual = 0.0
        else:
            # Circles do not intersect; closest point along baseline
            chosen = p_a + max(0.0, min(x, D)) * ex
            residual = math.sqrt(abs(h_sq))

        return float(chosen[0]), float(chosen[1]), float(residual)

    def solve_multilateration(
        self, active_anchors: List[str], distances: List[float]
    ) -> Tuple[float, float, float]:
        """
        N-anchor closed form least-squares trilateration (N >= 3).
        A * [x, y]^T = b
        """
        n = len(active_anchors)
        if n < 3:
            return 0.0, 0.0, 999.0

        coords = [self.anchors[a]["coords"] for a in active_anchors]
        x1, y1 = coords[0]
        r1 = distances[0]

        A_mat = []
        b_vec = []
        for i in range(1, n):
            xi, yi = coords[i]
            ri = distances[i]
            A_mat.append([2.0 * (xi - x1), 2.0 * (yi - y1)])
            b_vec.append(r1**2 - ri**2 + xi**2 + yi**2 - x1**2 - y1**2)

        A = np.array(A_mat)
        b = np.array(b_vec)

        try:
            sol, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
            res = float(residuals[0]) if len(residuals) > 0 else 0.0
            return float(sol[0]), float(sol[1]), res
        except np.linalg.LinAlgError:
            return float(x1), float(y1), 999.0

    def process_telemetry(self, msg: TelemetryMessage) -> PositionEstimate:
        """
        Core ingestion & estimation pipeline:
        1. Extract and filter RSSI readings for configured anchors.
        2. Detect sentinel -127 dBm and stale readings.
        3. Convert RSSI to estimated distances.
        4. Solve geometric observation (1D, 2D circle, or N-anchor).
        5. Fuse with IMU acceleration and gyro in the Adaptive Kalman Filter.
        6. Compute covariance-based uncertainty and return PositionEstimate.
        """
        now = time.time()
        dt = max(0.01, min(now - self.last_predict_time, 1.0))
        self.last_predict_time = now

        # -----------------------------------------------------------------
        # 1. RSSI Ingestion, Filtering & Sentinel Check
        # -----------------------------------------------------------------
        valid_anchor_ids = []
        filtered_distances = {}
        raw_rssi_map = {}

        for a_id, a_cfg in self.anchors.items():
            if not a_cfg["enabled"]:
                continue

            reading = msg.get_anchor(a_id)
            is_stale = (reading.age_ms > (self.stale_timeout_sec * 1000.0)) if (reading and reading.age_ms > 0) else False
            if reading is None or reading.is_missing or is_stale:
                raw_rssi_map[a_id] = None
                continue

            # Update filter
            raw_rssi_map[a_id] = reading.rssi_dbm
            flt = self.rssi_filters[a_id]
            filtered_rssi = flt.update(reading.rssi_dbm, now)

            dist = self.rssi_to_distance(filtered_rssi)
            filtered_distances[a_id] = dist
            valid_anchor_ids.append(a_id)

        # -----------------------------------------------------------------
        # 2. Geometric Observation Solver
        # -----------------------------------------------------------------
        geom_valid = False
        z_x, z_y = 0.0, 0.0
        geom_residual = 999.0
        method_name = "DEAD_RECKONING_IMU_ONLY"

        if "A" in valid_anchor_ids and "B" in valid_anchor_ids:
            d_a = filtered_distances["A"]
            d_b = filtered_distances["B"]

            if len(valid_anchor_ids) >= 3:
                # N >= 3 anchors available
                active_dists = [filtered_distances[aid] for aid in valid_anchor_ids]
                z_x, z_y, geom_residual = self.solve_multilateration(valid_anchor_ids, active_dists)
                method_name = "N_ANCHOR_TRILATERATION"
                geom_valid = True
            elif self.mode in ("1d_line", "adaptive_kalman"):
                # 1D line constraint between Anchor A and B
                z_x, z_y, geom_residual = self.solve_1d_line(d_a, d_b)
                method_name = "2_ANCHOR_1D_LINE"
                geom_valid = True
            else:
                # 2D circle intersection with prior disambiguation
                z_x, z_y, geom_residual = self.solve_2d_circles(d_a, d_b, self.state[1])
                method_name = "2_ANCHOR_CIRCLE_INTERSECT"
                geom_valid = True

        elif "A" in valid_anchor_ids or "B" in valid_anchor_ids:
            # Only single anchor visible
            single_id = valid_anchor_ids[0]
            dist = filtered_distances[single_id]
            pos = self.anchors[single_id]["coords"]
            # Rough single-anchor projection along baseline
            if single_id == "A":
                z_x = pos[0] + dist
            else:
                z_x = pos[0] - dist
            z_y = self.line_constraint_y
            method_name = f"SINGLE_ANCHOR_PROXIMITY_{single_id}"
            geom_valid = False  # Not fully determined

        # -----------------------------------------------------------------
        # 3. IMU Processing & ZUPT (Zero Velocity Update)
        # -----------------------------------------------------------------
        ax = msg.imu.accel_mps2.x - self.accel_bias_x
        ay = msg.imu.accel_mps2.y - self.accel_bias_y
        wz = msg.imu.gyro_rads.z

        # Update gyro heading integration
        self.heading_rad += wz * dt
        # Normalize heading to [-pi, pi]
        self.heading_rad = (self.heading_rad + math.pi) % (2.0 * math.pi) - math.pi

        # Stationary detection (Zero Velocity Update)
        linear_acc_mag = math.sqrt(ax**2 + ay**2)
        is_stationary = linear_acc_mag < self.zupt_threshold

        u = np.array([ax, ay]) if not is_stationary else np.array([0.0, 0.0])

        # -----------------------------------------------------------------
        # 4. Kalman Filter: Prediction Step
        # -----------------------------------------------------------------
        # Adjust dt in transition matrix
        F_k = self.F.copy()
        F_k[0, 2] = dt
        F_k[1, 3] = dt

        B_k = np.array([
            [0.5 * dt**2, 0.0],
            [0.0, 0.5 * dt**2],
            [dt, 0.0],
            [0.0, dt],
        ])

        x_pred = F_k @ self.state + B_k @ u
        P_pred = F_k @ self.P @ F_k.T + self.Q

        if is_stationary:
            # ZUPT: clamp predicted velocities to zero
            x_pred[2] = 0.0
            x_pred[3] = 0.0
            P_pred[2, 2] = 0.01
            P_pred[3, 3] = 0.01

        # -----------------------------------------------------------------
        # 5. Kalman Filter: Measurement Update (Adaptive R)
        # -----------------------------------------------------------------
        if geom_valid:
            if not self.has_first_fix:
                # Initialize state to first geometric fix
                self.state = np.array([z_x, z_y, 0.0, 0.0])
                self.P = np.diag([0.5, 0.5, 0.2, 0.2])
                self.has_first_fix = True
            else:
                # Determine adaptive measurement noise R
                R = self.R_nominal.copy()
                if self.adaptive_scaling:
                    # Adaptive factor based on residual and geometric condition
                    scale = 1.0
                    if geom_residual > self.spread_threshold:
                        scale *= self.high_noise_mult
                    elif geom_residual > 2.0:
                        scale *= 2.5

                    # If only 2 anchors, Y axis has higher uncertainty
                    if len(valid_anchor_ids) == 2:
                        R[1, 1] *= 4.0  # Underdetermined transverse axis

                    R *= scale

                # Kalman update
                z = np.array([z_x, z_y])
                innovation = z - self.H @ x_pred
                S = self.H @ P_pred @ self.H.T + R
                K = P_pred @ self.H.T @ np.linalg.inv(S)

                self.state = x_pred + K @ innovation
                self.P = (np.eye(4) - K @ self.H) @ P_pred
                method_name = f"ADAPTIVE_KALMAN_{method_name}"
        else:
            # No valid Wi-Fi measurement: propagate prediction only
            self.state = x_pred
            self.P = P_pred
            # Covariance grows naturally with process noise Q

        # -----------------------------------------------------------------
        # 6. Uncertainty & Output Formation
        # -----------------------------------------------------------------
        # Position uncertainty: 1-sigma standard deviation circle radius
        pos_uncertainty = float(math.sqrt(max(0.01, self.P[0, 0] + self.P[1, 1])))

        # Heading degrees
        heading_deg = math.degrees(self.heading_rad)

        estimate = PositionEstimate(
            device_id=msg.device_id,
            timestamp_ms=msg.timestamp_ms,
            x_m=float(self.state[0]),
            y_m=float(self.state[1]),
            heading_deg=heading_deg,
            method=method_name,
            valid=self.has_first_fix and (len(valid_anchor_ids) >= 2),
            uncertainty_m=pos_uncertainty,
            covariance={
                "p_xx": float(self.P[0, 0]),
                "p_yy": float(self.P[1, 1]),
                "p_vx": float(self.P[2, 2]),
                "p_vy": float(self.P[3, 3]),
            },
            anchor_distances={
                aid: filtered_distances.get(aid) for aid in self.anchors
            },
            raw_rssi=raw_rssi_map,
        )
        return estimate

    def reset(self, initial_x: float = 0.0, initial_y: float = 0.0):
        """Resets filter states."""
        self.state = np.array([initial_x, initial_y, 0.0, 0.0], dtype=float)
        self.P = np.diag([1.0, 1.0, 0.5, 0.5])
        self.has_first_fix = False
        self.heading_rad = 0.0
        for flt in self.rssi_filters.values():
            flt.history.clear()
            flt.ema_value = None
