"""
Integration test for the Web Server & REST / SSE endpoints.
"""

import json
import os
import time
import urllib.request
import unittest
import threading

from web.server import ThreadedHTTPServer, SystemState, make_request_handler


class TestServerLive(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        import tempfile
        cls.temp_dir = tempfile.TemporaryDirectory(ignore_cleanup_errors=True)
        cls.db_path = os.path.join(cls.temp_dir.name, "live_test.db")
        cls.config = {
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
            "storage": {"db_path": cls.db_path, "max_records": 100},
            "serial": {"port": "COM99", "simulation_fallback": True},
            "server": {"host": "127.0.0.1", "port": 8899},
        }
        cls.state = SystemState(cls.config, static_dir="web/static")
        cls.state.serial_service.start()

        handler = make_request_handler(cls.state)
        cls.server = ThreadedHTTPServer(("127.0.0.1", 8899), handler)
        cls.server_thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.server_thread.start()
        time.sleep(0.5)

    @classmethod
    def tearDownClass(cls):
        cls.state.serial_service.stop()
        cls.server.shutdown()
        cls.server.server_close()
        try:
            cls.temp_dir.cleanup()
        except Exception:
            pass

    def test_get_status_endpoint(self):
        url = "http://127.0.0.1:8899/api/status"
        with urllib.request.urlopen(url, timeout=3.0) as resp:
            self.assertEqual(resp.status, 200)
            data = json.loads(resp.read().decode())
            self.assertTrue(data["connected"])
            self.assertTrue(data["simulated"])
            self.assertIn("position", data)

    def test_post_telemetry_endpoint(self):
        url = "http://127.0.0.1:8899/api/telemetry"
        payload = json.dumps({
            "schema_version": 1,
            "type": "telemetry",
            "device_id": "tracker_01",
            "timestamp_ms": 99000,
            "imu": {"accel_mps2": {"x": 0.0, "y": 0.0, "z": 9.8}, "gyro_rads": {"x": 0, "y": 0, "z": 0}},
            "anchors": [
                {"anchor_id": "A", "ssid": "INDOOR_ANCHOR_A", "rssi_dbm": -45, "valid": True},
                {"anchor_id": "B", "ssid": "INDOOR_ANCHOR_B", "rssi_dbm": -70, "valid": True},
            ],
        }).encode("utf-8")

        req = urllib.request.Request(url, data=payload, headers={"Content-Type": "application/json"}, method="POST")
        with urllib.request.urlopen(req, timeout=3.0) as resp:
            self.assertEqual(resp.status, 200)
            data = json.loads(resp.read().decode())
            self.assertEqual(data["status"], "ok")


if __name__ == "__main__":
    unittest.main()
