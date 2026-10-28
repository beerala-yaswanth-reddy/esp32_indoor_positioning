"""
Test Suite: Position Estimation & Sensor Fusion
Tests Items 3, 4, 5, 6, 7 of Section 12.
"""

import json
import math
import unittest
from core.protocol import TelemetryMessage, IMUReading, AnchorReading, Vector3
from core.position_estimator import PositionEstimator, RSSIFilter


class TestPositionEstimator(unittest.TestCase):

    def setUp(self):
        self.config = {
            "anchors": [
                {"id": "A", "ssid": "INDOOR_ANCHOR_A", "coordinates": [0.0, 0.0], "enabled": True},
                {"id": "B", "ssid": "INDOOR_ANCHOR_B", "coordinates": [10.0, 0.0], "enabled": True},
            ],
            "estimator": {
                "mode": "adaptive_kalman",
                "d0_meters": 1.0,
                "rssi_0_dbm": -45.0,
                "path_loss_exponent": 2.4,
                "line_constraint_y": 0.0,
                "stale_timeout_ms": 2500,
                "kalman": {
                    "q_pos": 0.02,
                    "q_vel": 0.15,
                    "r_base": 3.0,
                    "adaptive_scaling": True,
                },
                "imu": {
                    "accel_bias_x": 0.0,
                    "accel_bias_y": 0.0,
                    "zupt_threshold_mps2": 0.25,
                },
            },
        }
        self.estimator = PositionEstimator(self.config)

    def test_rssi_to_distance_calibration(self):
        # At 1.0m, RSSI should be rssi_0 (-45 dBm)
        dist_1m = self.estimator.rssi_to_distance(-45.0)
        self.assertAlmostEqual(dist_1m, 1.0, places=2)

        # At greater distance, RSSI drops; distance should increase
        dist_drop = self.estimator.rssi_to_distance(-65.0)
        self.assertGreater(dist_drop, 1.0)
        # Expected: 1.0 * 10^(( -45 - (-65) ) / 24) = 10^(20/24) = 10^0.833 ~ 6.81m
        expected_dist = 1.0 * (10.0 ** (20.0 / 24.0))
        self.assertAlmostEqual(dist_drop, expected_dist, places=1)

    def test_rssi_noise_filtering(self):
        flt = RSSIFilter(window_size=5, ema_alpha=0.4)
        t = 100.0
        # Feed noisy samples around -50 dBm with a wild multipath outlier (-85 dBm)
        flt.update(-50.0, t)
        flt.update(-51.0, t + 0.1)
        val = flt.update(-85.0, t + 0.2)  # Wild spike
        # Median filter should mitigate the spike
        self.assertGreater(val, -65.0)

    def test_solve_1d_line(self):
        # Tracker exactly halfway between A (0,0) and B (10,0) at x=5.0
        # d_a = 5.0, d_b = 5.0
        x, y, res = self.estimator.solve_1d_line(5.0, 5.0)
        self.assertAlmostEqual(x, 5.0, places=2)
        self.assertAlmostEqual(y, 0.0, places=2)
        self.assertAlmostEqual(res, 0.0, places=2)

        # Tracker closer to A at x=2.0 -> d_a = 2.0, d_b = 8.0
        x2, y2, res2 = self.estimator.solve_1d_line(2.0, 8.0)
        self.assertAlmostEqual(x2, 2.0, places=2)
        self.assertAlmostEqual(y2, 0.0, places=2)

    def test_underdetermined_2d_handling(self):
        # Two circle intersection with prior disambiguation
        x, y, res = self.estimator.solve_2d_circles(5.0, 5.0, prior_y=0.0)
        self.assertAlmostEqual(x, 5.0, places=1)
        self.assertAlmostEqual(res, 0.0, places=1)

    def test_zupt_stationary_zero_drift(self):
        """Zero Velocity Update must clamp velocity when sensor acceleration is below threshold."""
        # Send 10 stationary packets
        for i in range(10):
            msg = TelemetryMessage(
                device_id="tracker_01",
                timestamp_ms=1000 + i * 100,
                imu=IMUReading(
                    accel_mps2=Vector3(0.02, -0.01, 9.80),  # Under zupt threshold (0.25)
                    gyro_rads=Vector3(0.0, 0.0, 0.0),
                ),
                anchors=[
                    AnchorReading("A", "INDOOR_ANCHOR_A", -55.0, True),
                    AnchorReading("B", "INDOOR_ANCHOR_B", -55.0, True),
                ],
            )
            est = self.estimator.process_telemetry(msg)

        # Velocity states should remain clamped to 0.0, not accumulating drift
        self.assertAlmostEqual(self.estimator.state[2], 0.0, places=2)
        self.assertAlmostEqual(self.estimator.state[3], 0.0, places=2)

    def test_missing_single_anchor_fallback(self):
        """When only Anchor A is detected, system flags invalid/degraded rather than false precision."""
        msg = TelemetryMessage(
            device_id="tracker_01",
            timestamp_ms=2000,
            imu=IMUReading(),
            anchors=[
                AnchorReading("A", "INDOOR_ANCHOR_A", -48.0, True),
                AnchorReading("B", "INDOOR_ANCHOR_B", -127.0, False),  # Missing Sentinel
            ],
        )
        est = self.estimator.process_telemetry(msg)
        self.assertFalse(est.valid)
        self.assertIn("SINGLE_ANCHOR", est.method)

    def test_stale_rssi_observation(self):
        """When RSSI measurement age exceeds stale_timeout_ms (2500ms), it is rejected as stale."""
        # Anchor B reading has age_ms = 4000 (> 2500 timeout)
        msg = TelemetryMessage(
            device_id="tracker_01",
            timestamp_ms=3000,
            imu=IMUReading(),
            anchors=[
                AnchorReading("A", "INDOOR_ANCHOR_A", -48.0, True, age_ms=50),
                AnchorReading("B", "INDOOR_ANCHOR_B", -52.0, True, age_ms=4000),  # Stale!
            ],
        )
        est = self.estimator.process_telemetry(msg)
        # Anchor B should have been discarded due to staleness
        self.assertIsNone(est.raw_rssi.get("B"))
        self.assertFalse(est.valid)
        self.assertIn("SINGLE_ANCHOR", est.method)

    def test_three_anchor_multilateration(self):
        """Verify N-anchor multilateration when 3 anchors are configured and active."""
        config_3 = {
            "anchors": [
                {"id": "A", "ssid": "INDOOR_ANCHOR_A", "coordinates": [0.0, 0.0], "enabled": True},
                {"id": "B", "ssid": "INDOOR_ANCHOR_B", "coordinates": [10.0, 0.0], "enabled": True},
                {"id": "C", "ssid": "INDOOR_ANCHOR_C", "coordinates": [5.0, 8.66], "enabled": True},
            ],
            "estimator": {
                "mode": "adaptive_kalman",
                "d0_meters": 1.0,
                "rssi_0_dbm": -45.0,
                "path_loss_exponent": 2.0,
                "stale_timeout_ms": 2500,
                "kalman": {"q_pos": 0.02, "q_vel": 0.15, "r_base": 3.0, "adaptive_scaling": True},
                "imu": {"accel_bias_x": 0.0, "accel_bias_y": 0.0, "zupt_threshold_mps2": 0.25},
            },
        }
        est_3 = PositionEstimator(config_3)
        # Target at (5.0, 2.887)
        # Distance to A: sqrt(5^2 + 2.887^2) = 5.77m
        # Distance to B: sqrt(5^2 + 2.887^2) = 5.77m
        # Distance to C: sqrt(0^2 + (8.66 - 2.887)^2) = 5.773m
        sol_x, sol_y, res = est_3.solve_multilateration(["A", "B", "C"], [5.77, 5.77, 5.773])
        self.assertAlmostEqual(sol_x, 5.0, places=1)
        self.assertAlmostEqual(sol_y, 2.89, places=1)

    def test_imu_bias_correction(self):
        """Verify IMU acceleration bias subtraction."""
        self.estimator.accel_bias_x = 0.50
        self.estimator.accel_bias_y = -0.30
        msg = TelemetryMessage(
            device_id="tracker_01",
            timestamp_ms=4000,
            imu=IMUReading(
                accel_mps2=Vector3(x=0.50, y=-0.30, z=9.81),  # Exactly matches bias
                gyro_rads=Vector3(0, 0, 0),
            ),
            anchors=[
                AnchorReading("A", "INDOOR_ANCHOR_A", -48.0, True),
                AnchorReading("B", "INDOOR_ANCHOR_B", -52.0, True),
            ],
        )
        est = self.estimator.process_telemetry(msg)
        # After subtracting bias, net acceleration is ~0, triggering ZUPT
        self.assertAlmostEqual(self.estimator.state[2], 0.0, places=2)
        self.assertAlmostEqual(self.estimator.state[3], 0.0, places=2)


if __name__ == "__main__":
    unittest.main()
