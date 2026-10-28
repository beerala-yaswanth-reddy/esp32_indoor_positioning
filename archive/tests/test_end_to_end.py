"""
Test Suite: End-to-End Integration
Tests Item 10, 11, 12 of Section 12.
"""

import json
import os
import tempfile
import time
import unittest

from core.protocol import parse_telemetry_json
from core.position_estimator import PositionEstimator
from core.storage import StorageManager
from core.serial_ingestion import MockHardwareGenerator


class TestEndToEnd(unittest.TestCase):

    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory(ignore_cleanup_errors=True)
        self.db_path = os.path.join(self.temp_dir.name, "e2e_test.db")
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
                "stale_timeout_ms": 2500,
                "kalman": {"q_pos": 0.02, "q_vel": 0.15, "r_base": 3.0, "adaptive_scaling": True},
                "imu": {"accel_bias_x": 0.0, "accel_bias_y": 0.0, "zupt_threshold_mps2": 0.25},
            },
            "storage": {"db_path": self.db_path, "max_records": 1000},
        }
        self.storage = StorageManager(self.db_path)
        self.estimator = PositionEstimator(self.config)
        self.generator = MockHardwareGenerator(self.config)

    def tearDown(self):
        try:
            self.temp_dir.cleanup()
        except Exception:
            pass

    def test_pipeline_stream_integration(self):
        """Simulate a continuous 2-second stream (20 samples at 10 Hz) through the full pipeline."""
        estimates = []
        for _ in range(20):
            raw_line = self.generator.generate_sample()
            msg, err = parse_telemetry_json(raw_line)
            self.assertIsNone(err)
            self.assertIsNotNone(msg)

            # Estimate
            est = self.estimator.process_telemetry(msg)
            self.assertIsNotNone(est)
            estimates.append(est)

            # Persist
            self.storage.log_telemetry(msg)
            self.storage.log_position(est)

        # Check database records
        positions_logged = self.storage.query_positions(limit=50)
        self.assertEqual(len(positions_logged), 20)

        # Check estimate validity
        valid_count = sum(1 for e in estimates if e.valid)
        self.assertGreaterEqual(valid_count, 15)

        # Coordinates should remain within physical baseline bounds (around 0m to 10m)
        for e in estimates:
            if e.valid and e.x_m is not None:
                self.assertGreaterEqual(e.x_m, -2.0)
                self.assertLessEqual(e.x_m, 12.0)


if __name__ == "__main__":
    unittest.main()
