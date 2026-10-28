"""
Test Suite: Data Protocol & Sentinel Handling
Tests Item 1, 2, 3 of Section 12.
"""

import json
import unittest
from core.protocol import (
    parse_telemetry_json,
    SENTINEL_RSSI_MISSING,
    MIN_PLAUSIBLE_RSSI,
    MAX_PLAUSIBLE_RSSI,
)


class TestProtocol(unittest.TestCase):

    def test_valid_telemetry_payload(self):
        sample = {
            "schema_version": 1,
            "type": "telemetry",
            "device_id": "tracker_01",
            "timestamp_ms": 12500,
            "imu": {
                "accel_mps2": {"x": 0.12, "y": -0.04, "z": 9.72},
                "gyro_rads": {"x": 0.01, "y": 0.02, "z": -0.01},
                "valid": True,
            },
            "anchors": [
                {"anchor_id": "A", "ssid": "INDOOR_ANCHOR_A", "rssi_dbm": -48, "valid": True},
                {"anchor_id": "B", "ssid": "INDOOR_ANCHOR_B", "rssi_dbm": -65, "valid": True},
            ],
        }
        msg, err = parse_telemetry_json(json.dumps(sample))
        self.assertIsNone(err)
        self.assertIsNotNone(msg)
        self.assertEqual(msg.device_id, "tracker_01")
        self.assertEqual(msg.timestamp_ms, 12500)
        self.assertAlmostEqual(msg.imu.accel_mps2.z, 9.72)

        anchor_a = msg.get_anchor("A")
        self.assertIsNotNone(anchor_a)
        self.assertEqual(anchor_a.rssi_dbm, -48)
        self.assertFalse(anchor_a.is_missing)

    def test_sentinel_rssi_missing(self):
        """Verify -127 dBm sentinel is recognized as missing observation, not real RSSI."""
        sample = {
            "schema_version": 1,
            "type": "telemetry",
            "device_id": "tracker_01",
            "timestamp_ms": 13000,
            "imu": {"accel_mps2": {"x": 0, "y": 0, "z": 9.8}, "gyro_rads": {"x": 0, "y": 0, "z": 0}},
            "anchors": [
                {"anchor_id": "A", "ssid": "INDOOR_ANCHOR_A", "rssi_dbm": -48, "valid": True},
                {"anchor_id": "B", "ssid": "INDOOR_ANCHOR_B", "rssi_dbm": -127, "valid": True},
            ],
        }
        msg, err = parse_telemetry_json(json.dumps(sample))
        self.assertIsNone(err)
        self.assertIsNotNone(msg)

        anchor_b = msg.get_anchor("B")
        self.assertIsNotNone(anchor_b)
        self.assertTrue(anchor_b.is_missing)
        # Serialization should report null rssi_dbm
        self.assertIsNone(anchor_b.to_dict()["rssi_dbm"])
        self.assertFalse(anchor_b.to_dict()["valid"])

    def test_null_rssi(self):
        sample = {
            "schema_version": 1,
            "type": "telemetry",
            "device_id": "tracker_01",
            "timestamp_ms": 13100,
            "imu": {"accel_mps2": {"x": 0, "y": 0, "z": 9.8}, "gyro_rads": {"x": 0, "y": 0, "z": 0}},
            "anchors": [
                {"anchor_id": "A", "ssid": "INDOOR_ANCHOR_A", "rssi_dbm": None, "valid": False},
            ],
        }
        msg, err = parse_telemetry_json(json.dumps(sample))
        self.assertIsNone(err)
        anchor_a = msg.get_anchor("A")
        self.assertTrue(anchor_a.is_missing)

    def test_invalid_payloads(self):
        # Empty line
        msg, err = parse_telemetry_json("")
        self.assertIsNone(msg)
        self.assertIn("Empty", err)

        # Truncated JSON
        msg, err = parse_telemetry_json('{"device_id": "tracker_01", "timestamp_ms": ')
        self.assertIsNone(msg)
        self.assertIsNotNone(err)

        # Corrupted characters
        msg, err = parse_telemetry_json("RANDOM_GARBAGE_SERIAL_OUTPUT_12345")
        self.assertIsNone(msg)
        self.assertIsNotNone(err)


if __name__ == "__main__":
    unittest.main()
