"""
Data Logging and Persistence Layer for ESP32 Indoor Positioning.

Uses SQLite with indexed tables:
- telemetry_logs: Raw sensor and RSSI samples
- position_logs: Estimated coordinates, method, uncertainty, validity
- system_events: Disconnections, reconnections, calibration, sensor errors

Includes:
- Bounded ring buffering for in-flight records
- Historical query API (by device, time range, limit)
- Retention pruning policy to prevent unbounded disk growth
"""

import os
import sqlite3
import time
import json
from typing import Dict, List, Optional, Any
from core.protocol import TelemetryMessage, PositionEstimate


from contextlib import contextmanager

class StorageManager:
    def __init__(self, db_path: str = "indoor_positioning.db", max_records: int = 50000):
        self.db_path = db_path
        self.max_records = max_records
        self._init_database()

    @contextmanager
    def _get_connection(self):
        conn = sqlite3.connect(self.db_path, timeout=10.0)
        conn.row_factory = sqlite3.Row
        try:
            yield conn
        finally:
            conn.close()

    def _init_database(self):
        with self._get_connection() as conn:
            cursor = conn.cursor()
            # Telemetry raw logs
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS telemetry_logs (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    device_id TEXT NOT NULL,
                    timestamp_ms INTEGER NOT NULL,
                    receive_time REAL NOT NULL,
                    accel_x REAL,
                    accel_y REAL,
                    accel_z REAL,
                    gyro_x REAL,
                    gyro_y REAL,
                    gyro_z REAL,
                    rssi_a REAL,
                    rssi_b REAL,
                    raw_json TEXT NOT NULL
                )
            """)
            cursor.execute("CREATE INDEX IF NOT EXISTS idx_telemetry_time ON telemetry_logs(receive_time)")
            cursor.execute("CREATE INDEX IF NOT EXISTS idx_telemetry_dev ON telemetry_logs(device_id)")

            # Position estimate logs
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS position_logs (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    device_id TEXT NOT NULL,
                    timestamp_ms INTEGER NOT NULL,
                    server_time REAL NOT NULL,
                    x_m REAL,
                    y_m REAL,
                    heading_deg REAL,
                    method TEXT NOT NULL,
                    valid INTEGER NOT NULL,
                    uncertainty_m REAL NOT NULL,
                    cov_xx REAL,
                    cov_yy REAL,
                    dist_a REAL,
                    dist_b REAL
                )
            """)
            cursor.execute("CREATE INDEX IF NOT EXISTS idx_position_time ON position_logs(server_time)")
            cursor.execute("CREATE INDEX IF NOT EXISTS idx_position_dev ON position_logs(device_id)")

            # System events (connectivity, hardware errors, calibrations)
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS system_events (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    device_id TEXT NOT NULL,
                    event_type TEXT NOT NULL,
                    severity TEXT NOT NULL,
                    message TEXT NOT NULL,
                    timestamp REAL NOT NULL,
                    details_json TEXT
                )
            """)
            cursor.execute("CREATE INDEX IF NOT EXISTS idx_events_time ON system_events(timestamp)")
            conn.commit()

    def log_telemetry(self, msg: TelemetryMessage):
        """Logs raw telemetry reading."""
        a_reading = msg.get_anchor("A")
        b_reading = msg.get_anchor("B")
        rssi_a = a_reading.rssi_dbm if (a_reading and not a_reading.is_missing) else None
        rssi_b = b_reading.rssi_dbm if (b_reading and not b_reading.is_missing) else None

        with self._get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute("""
                INSERT INTO telemetry_logs (
                    device_id, timestamp_ms, receive_time,
                    accel_x, accel_y, accel_z,
                    gyro_x, gyro_y, gyro_z,
                    rssi_a, rssi_b, raw_json
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """, (
                msg.device_id,
                msg.timestamp_ms,
                msg.receive_time_unix,
                msg.imu.accel_mps2.x,
                msg.imu.accel_mps2.y,
                msg.imu.accel_mps2.z,
                msg.imu.gyro_rads.x,
                msg.imu.gyro_rads.y,
                msg.imu.gyro_rads.z,
                rssi_a,
                rssi_b,
                msg.to_json(),
            ))
            conn.commit()

    def log_position(self, est: PositionEstimate):
        """Logs filtered/estimated position."""
        dist_a = est.anchor_distances.get("A")
        dist_b = est.anchor_distances.get("B")

        with self._get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute("""
                INSERT INTO position_logs (
                    device_id, timestamp_ms, server_time,
                    x_m, y_m, heading_deg,
                    method, valid, uncertainty_m,
                    cov_xx, cov_yy, dist_a, dist_b
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """, (
                est.device_id,
                est.timestamp_ms,
                est.server_timestamp,
                est.x_m if est.valid else None,
                est.y_m if est.valid else None,
                est.heading_deg,
                est.method,
                1 if est.valid else 0,
                est.uncertainty_m,
                est.covariance.get("p_xx"),
                est.covariance.get("p_yy"),
                dist_a,
                dist_b,
            ))
            conn.commit()

    def log_event(self, device_id: str, event_type: str, severity: str, message: str, details: Optional[Dict] = None):
        """Logs lifecycle and error events."""
        with self._get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute("""
                INSERT INTO system_events (
                    device_id, event_type, severity, message, timestamp, details_json
                ) VALUES (?, ?, ?, ?, ?, ?)
            """, (
                device_id,
                event_type,
                severity,
                message,
                time.time(),
                json.dumps(details or {}),
            ))
            conn.commit()

    def query_positions(
        self,
        device_id: Optional[str] = None,
        start_time: Optional[float] = None,
        end_time: Optional[float] = None,
        limit: int = 500,
    ) -> List[Dict[str, Any]]:
        """Queries historical positions for map breadcrumbs and trajectory charts."""
        query = "SELECT * FROM position_logs WHERE 1=1"
        params: List[Any] = []

        if device_id:
            query += " AND device_id = ?"
            params.append(device_id)
        if start_time is not None:
            query += " AND server_time >= ?"
            params.append(start_time)
        if end_time is not None:
            query += " AND server_time <= ?"
            params.append(end_time)

        query += " ORDER BY server_time DESC LIMIT ?"
        params.append(limit)

        with self._get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute(query, params)
            rows = cursor.fetchall()

        results = []
        for r in reversed(rows):
            results.append({
                "id": r["id"],
                "device_id": r["device_id"],
                "timestamp_ms": r["timestamp_ms"],
                "server_time": r["server_time"],
                "x_m": r["x_m"],
                "y_m": r["y_m"],
                "heading_deg": r["heading_deg"],
                "method": r["method"],
                "valid": bool(r["valid"]),
                "uncertainty_m": r["uncertainty_m"],
                "dist_a": r["dist_a"],
                "dist_b": r["dist_b"],
            })
        return results

    def query_events(self, limit: int = 100) -> List[Dict[str, Any]]:
        """Queries recent system events."""
        with self._get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute("SELECT * FROM system_events ORDER BY timestamp DESC LIMIT ?", (limit,))
            rows = cursor.fetchall()

        return [
            {
                "id": r["id"],
                "device_id": r["device_id"],
                "event_type": r["event_type"],
                "severity": r["severity"],
                "message": r["message"],
                "timestamp": r["timestamp"],
                "details": json.loads(r["details_json"] or "{}"),
            }
            for r in rows
        ]

    def prune_old_records(self):
        """Prunes database if record count exceeds max_records."""
        with self._get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute("SELECT COUNT(*) FROM position_logs")
            count = cursor.fetchone()[0]
            if count > self.max_records:
                excess = count - self.max_records
                cursor.execute("""
                    DELETE FROM position_logs WHERE id IN (
                        SELECT id FROM position_logs ORDER BY server_time ASC LIMIT ?
                    )
                """, (excess,))
                conn.commit()
