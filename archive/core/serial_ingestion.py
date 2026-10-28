"""
Hardware Communication & Serial Ingestion Layer.

Features:
- USB Serial communication (pyserial) with auto-reconnection
- Fallback high-fidelity Simulated Hardware Stream when ESP32 is not plugged in
- Thread-safe non-blocking ingestion loop
- Callback architecture to forward validated TelemetryMessage to estimator & storage
- Sentinel detection, buffer protection, and health metrics
"""

import math
import random
import sys
import threading
import time
from typing import Callable, Dict, Optional, Any

import serial
import serial.tools.list_ports

from core.protocol import (
    TelemetryMessage,
    IMUReading,
    AnchorReading,
    Vector3,
    parse_telemetry_json,
    SENTINEL_RSSI_MISSING,
)


class MockHardwareGenerator:
    """
    Generates realistic ESP32-WROVER telemetry when physical hardware is not attached.
    Simulates a tracker moving along a 10m path between Anchor A and Anchor B,
    calculating realistic log-distance RSSI, noise, and MPU6050 accelerometer/gyro readings.
    """

    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.t = 0.0
        self.dt = 0.1
        self.rssi_0 = config.get("estimator", {}).get("rssi_0_dbm", -45.0)
        self.ple = config.get("estimator", {}).get("path_loss_exponent", 2.4)
        self.device_id = "tracker_01"

    def generate_sample(self) -> str:
        self.t += self.dt
        # Simulated movement back and forth between x=1.0 and x=9.0 meters
        # Period = 20s
        phase = (self.t % 20.0) / 20.0
        x_true = 5.0 + 4.0 * math.sin(2 * math.pi * phase)
        y_true = 0.0  # along 1D line

        # Velocity and acceleration
        vx_true = 4.0 * (2 * math.pi / 20.0) * math.cos(2 * math.pi * phase)
        ax_true = -4.0 * ((2 * math.pi / 20.0) ** 2) * math.sin(2 * math.pi * phase)

        # Distances to anchors
        # Anchor A at (0, 0), Anchor B at (10, 0)
        dist_a = max(0.2, abs(x_true - 0.0))
        dist_b = max(0.2, abs(10.0 - x_true))

        # RSSI with log-normal shadowing noise
        noise_a = random.gauss(0.0, 2.0)
        noise_b = random.gauss(0.0, 2.0)

        # Deliberate periodic temporary signal drop (e.g. 5% chance of missing anchor B)
        drop_b = random.random() < 0.04

        rssi_a = round(self.rssi_0 - 10.0 * self.ple * math.log10(dist_a) + noise_a, 1)
        rssi_b = SENTINEL_RSSI_MISSING if drop_b else round(self.rssi_0 - 10.0 * self.ple * math.log10(dist_b) + noise_b, 1)

        # MPU6050 reading with noise and gravity (z = 9.81 m/s^2)
        accel_x = round(ax_true + random.gauss(0.0, 0.15), 3)
        accel_y = round(random.gauss(0.0, 0.10), 3)
        accel_z = round(9.806 + random.gauss(0.0, 0.15), 3)

        gyro_x = round(random.gauss(0.0, 0.01), 3)
        gyro_y = round(random.gauss(0.0, 0.01), 3)
        gyro_z = round(random.gauss(0.0, 0.02), 3)

        timestamp_ms = int(self.t * 1000)

        payload = {
            "schema_version": 1,
            "type": "telemetry",
            "device_id": self.device_id,
            "timestamp_ms": timestamp_ms,
            "imu": {
                "accel_mps2": {"x": accel_x, "y": accel_y, "z": accel_z},
                "gyro_rads": {"x": gyro_x, "y": gyro_y, "z": gyro_z},
                "valid": True,
                "temp_c": 28.5,
            },
            "anchors": [
                {
                    "anchor_id": "A",
                    "ssid": "INDOOR_ANCHOR_A",
                    "rssi_dbm": rssi_a,
                    "valid": True,
                    "age_ms": int(random.uniform(20, 80)),
                },
                {
                    "anchor_id": "B",
                    "ssid": "INDOOR_ANCHOR_B",
                    "rssi_dbm": rssi_b,
                    "valid": not drop_b,
                    "age_ms": int(random.uniform(20, 80)),
                },
            ],
            "status": {
                "simulation": True,
                "battery_v": 3.95,
                "free_heap": 182400,
            },
        }
        import json
        return json.dumps(payload)


class SerialIngestionService:
    """
    Manages serial connection to ESP32-WROVER.
    Automatically enters mock simulation mode if the physical port is not available,
    and transitions smoothly back to physical hardware when plugged in.
    """

    def __init__(
        self,
        config: Dict[str, Any],
        on_telemetry_callback: Callable[[TelemetryMessage], None],
        on_event_callback: Optional[Callable[[str, str, str], None]] = None,
    ):
        self.config = config
        self.on_telemetry = on_telemetry_callback
        self.on_event = on_event_callback
        self.serial_cfg = config.get("serial", {})
        self.port_name = self.serial_cfg.get("port", "COM3")
        self.baud_rate = int(self.serial_cfg.get("baud_rate", 115200))
        self.timeout = float(self.serial_cfg.get("timeout_sec", 1.0))
        self.simulation_fallback = bool(self.serial_cfg.get("simulation_fallback", True))

        self.serial_conn: Optional[serial.Serial] = None
        self.is_running = False
        self.is_connected = False
        self.is_simulated = False
        self.thread: Optional[threading.Thread] = None

        self.mock_generator = MockHardwareGenerator(config)
        self.messages_received = 0
        self.errors_count = 0

    @staticmethod
    def list_available_ports():
        """Lists available COM ports on the system."""
        ports = serial.tools.list_ports.comports()
        return [{"device": p.device, "description": p.description} for p in ports]

    def start(self):
        """Starts ingestion thread."""
        self.is_running = True
        self.thread = threading.Thread(target=self._run_loop, daemon=True, name="SerialIngestionWorker")
        self.thread.start()

    def stop(self):
        """Stops ingestion thread and closes port."""
        self.is_running = False
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.close()
            except Exception:
                pass
        self.is_connected = False

    def _notify_event(self, event_type: str, severity: str, message: str):
        if self.on_event:
            self.on_event(event_type, severity, message)

    def _run_loop(self):
        while self.is_running:
            # Attempt to open real serial port
            try:
                available = [p.device for p in serial.tools.list_ports.comports()]
                if self.port_name in available:
                    self.serial_conn = serial.Serial(
                        port=self.port_name,
                        baudrate=self.baud_rate,
                        timeout=self.timeout,
                    )
                    self.is_connected = True
                    self.is_simulated = False
                    self._notify_event("DEVICE_CONNECTED", "INFO", f"Connected to physical ESP32 on {self.port_name}")

                    # Read loop
                    while self.is_running and self.serial_conn.is_open:
                        line = self.serial_conn.readline().decode("utf-8", errors="replace").strip()
                        if line:
                            msg, err = parse_telemetry_json(line)
                            if msg:
                                self.messages_received += 1
                                self.on_telemetry(msg)
                            else:
                                self.errors_count += 1
                else:
                    # Physical port not present
                    if self.simulation_fallback:
                        if not self.is_simulated:
                            self.is_simulated = True
                            self.is_connected = True
                            self._notify_event(
                                "SIMULATION_MODE",
                                "WARN",
                                f"Port {self.port_name} not detected. Falling back to active hardware simulation stream.",
                            )

                        # Generate synthetic sample at 10 Hz
                        mock_json = self.mock_generator.generate_sample()
                        msg, err = parse_telemetry_json(mock_json)
                        if msg:
                            self.messages_received += 1
                            self.on_telemetry(msg)
                        time.sleep(0.1)
                    else:
                        time.sleep(1.0)

            except (serial.SerialException, OSError) as e:
                self.is_connected = False
                self._notify_event("DEVICE_DISCONNECTED", "ERROR", f"Serial connection error: {e}")
                if self.serial_conn:
                    try:
                        self.serial_conn.close()
                    except Exception:
                        pass
                time.sleep(1.0)
            except Exception as ex:
                self.errors_count += 1
                time.sleep(0.5)
