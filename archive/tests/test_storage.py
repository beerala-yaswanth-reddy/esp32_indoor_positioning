"""
Test Suite: Data Persistence & Historical Queries
Tests Items 8, 9, 11 of Section 12.
"""

import os
import tempfile
import time
import unittest

from core.protocol import TelemetryMessage, PositionEstimate, IMUReading, AnchorReading, Vector3
from core.storage import StorageManager


class TestStorage(unittest.TestCase):

    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory(ignore_cleanup_errors=True)
        self.db_path = os.path.join(self.temp_dir.name, "test_indoor.db")
        self.storage = StorageManager(db_path=self.db_path, max_records=100)

    def tearDown(self):
        try:
            self.temp_dir.cleanup()
        except Exception:
            pass

    def test_log_and_query_telemetry(self):
        msg = TelemetryMessage(
            device_id="tracker_01",
            timestamp_ms=5000,
            imu=IMUReading(
                accel_mps2=Vector3(0.1, 0.2, 9.8),
                gyro_rads=Vector3(0.01, 0.02, 0.03),
            ),
            anchors=[
                AnchorReading("A", "INDOOR_ANCHOR_A", -48.0, True),
                AnchorReading("B", "INDOOR_ANCHOR_B", -62.0, True),
            ],
        )
        self.storage.log_telemetry(msg)

        # Query events or verify DB table
        with self.storage._get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute("SELECT COUNT(*) FROM telemetry_logs WHERE device_id = 'tracker_01'")
            count = cursor.fetchone()[0]
            self.assertEqual(count, 1)

    def test_log_and_query_positions(self):
        now = time.time()
        for i in range(5):
            est = PositionEstimate(
                device_id="tracker_01",
                timestamp_ms=1000 + i * 100,
                x_m=float(i),
                y_m=0.0,
                heading_deg=10.0 * i,
                method="ADAPTIVE_KALMAN_1D_LINE",
                valid=True,
                uncertainty_m=0.45,
                server_timestamp=now + i * 0.1,
            )
            self.storage.log_position(est)

        # Query all
        results = self.storage.query_positions(device_id="tracker_01", limit=10)
        self.assertEqual(len(results), 5)
        self.assertAlmostEqual(results[0]["x_m"], 0.0)
        self.assertAlmostEqual(results[-1]["x_m"], 4.0)

    def test_event_logging(self):
        self.storage.log_event("tracker_01", "DEVICE_CONNECTED", "INFO", "USB connection established")
        events = self.storage.query_events(limit=10)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]["event_type"], "DEVICE_CONNECTED")
        self.assertEqual(events[0]["severity"], "INFO")

    def test_historical_time_range_query(self):
        base_t = 1000.0
        for i in range(10):
            est = PositionEstimate(
                device_id="tracker_01",
                timestamp_ms=1000 + i * 100,
                x_m=float(i),
                y_m=0.0,
                heading_deg=0.0,
                method="TEST",
                valid=True,
                uncertainty_m=0.5,
                server_timestamp=base_t + i * 1.0,
            )
            self.storage.log_position(est)

        # Query range from base_t + 2.0 to base_t + 5.0 (indices 2, 3, 4, 5 -> 4 records)
        ranged = self.storage.query_positions(
            device_id="tracker_01",
            start_time=base_t + 2.0,
            end_time=base_t + 5.0,
        )
        self.assertEqual(len(ranged), 4)
        self.assertAlmostEqual(ranged[0]["x_m"], 2.0)
        self.assertAlmostEqual(ranged[-1]["x_m"], 5.0)

    def test_prune_old_records(self):
        # Storage max_records was set to 100 in setUp.
        # Let's insert 120 records and prune.
        for i in range(120):
            est = PositionEstimate(
                device_id="tracker_01",
                timestamp_ms=i * 100,
                x_m=1.0,
                y_m=1.0,
                heading_deg=0.0,
                method="PRUNE_TEST",
                valid=True,
                uncertainty_m=0.5,
                server_timestamp=1000.0 + i,
            )
            self.storage.log_position(est)

        self.storage.prune_old_records()
        with self.storage._get_connection() as conn:
            count = conn.cursor().execute("SELECT COUNT(*) FROM position_logs").fetchone()[0]
            self.assertLessEqual(count, 100)


if __name__ == "__main__":
    unittest.main()
