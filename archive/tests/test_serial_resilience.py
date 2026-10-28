"""
Test Suite: Serial Communication Resilience, Reconnection & Recovery
Tests Items 8 & 11 of Section 12.
"""

import time
import unittest
from core.serial_ingestion import SerialIngestionService, MockHardwareGenerator
from core.protocol import parse_telemetry_json


class TestSerialResilience(unittest.TestCase):

    def setUp(self):
        self.config = {
            "serial": {
                "port": "COM_NON_EXISTENT_99",
                "baud_rate": 115200,
                "timeout_sec": 0.2,
                "simulation_fallback": True,
            },
            "estimator": {
                "rssi_0_dbm": -45.0,
                "path_loss_exponent": 2.4,
            },
        }
        self.received_messages = []
        self.events_logged = []

    def on_telemetry(self, msg):
        self.received_messages.append(msg)

    def on_event(self, event_type, severity, message):
        self.events_logged.append((event_type, severity, message))

    def test_simulation_fallback_and_event_dispatch(self):
        """When hardware COM port is not attached, service triggers simulation fallback event cleanly."""
        service = SerialIngestionService(
            config=self.config,
            on_telemetry_callback=self.on_telemetry,
            on_event_callback=self.on_event,
        )
        service.start()
        # Give thread 0.35s to execute at least a few cycles
        time.sleep(0.35)
        service.stop()

        self.assertTrue(service.is_simulated)
        self.assertGreater(len(self.received_messages), 0)
        self.assertGreater(len(self.events_logged), 0)
        # Event type must include SIMULATION_MODE
        event_types = [ev[0] for ev in self.events_logged]
        self.assertIn("SIMULATION_MODE", event_types)

    def test_mock_hardware_generator_bounds(self):
        """Mock hardware generator must output valid JSON payloads conforming to schema."""
        gen = MockHardwareGenerator(self.config)
        for _ in range(10):
            sample = gen.generate_sample()
            msg, err = parse_telemetry_json(sample)
            self.assertIsNone(err)
            self.assertIsNotNone(msg)
            self.assertEqual(msg.device_id, "tracker_01")
            self.assertGreaterEqual(msg.imu.accel_mps2.z, 8.0)  # Near 1G gravity


if __name__ == "__main__":
    unittest.main()
