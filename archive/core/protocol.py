"""
Standardized Data Protocol and Message Serialization for ESP32 Indoor Positioning.

Handles:
- Incoming newline-delimited JSON telemetry parsing from USB serial / Wi-Fi
- Sentinel handling (e.g. -127 dBm sentinel is recognized as missing observation)
- Outgoing position estimation payload generation
- Validation, bounds checking, and schema enforcement
"""

import json
import time
from dataclasses import dataclass, field, asdict
from typing import List, Optional, Dict, Any, Tuple

SCHEMA_VERSION = 1
SENTINEL_RSSI_MISSING = -127.0
MIN_PLAUSIBLE_RSSI = -105.0
MAX_PLAUSIBLE_RSSI = -10.0


@dataclass
class Vector3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def to_dict(self) -> Dict[str, float]:
        return {"x": round(self.x, 4), "y": round(self.y, 4), "z": round(self.z, 4)}


@dataclass
class IMUReading:
    accel_mps2: Vector3 = field(default_factory=Vector3)
    gyro_rads: Vector3 = field(default_factory=Vector3)
    valid: bool = True
    temp_c: Optional[float] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "accel_mps2": self.accel_mps2.to_dict(),
            "gyro_rads": self.gyro_rads.to_dict(),
            "valid": self.valid,
            "temp_c": self.temp_c,
        }


@dataclass
class AnchorReading:
    anchor_id: str
    ssid: str
    rssi_dbm: float
    valid: bool = True
    age_ms: int = 0

    @property
    def is_missing(self) -> bool:
        """Sentinel check: -127 dBm or outside plausible Wi-Fi RSSI bounds."""
        return (
            not self.valid
            or self.rssi_dbm <= SENTINEL_RSSI_MISSING
            or self.rssi_dbm < MIN_PLAUSIBLE_RSSI
            or self.rssi_dbm > MAX_PLAUSIBLE_RSSI
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "anchor_id": self.anchor_id,
            "ssid": self.ssid,
            "rssi_dbm": None if self.is_missing else round(self.rssi_dbm, 1),
            "valid": not self.is_missing,
            "age_ms": self.age_ms,
        }


@dataclass
class TelemetryMessage:
    device_id: str
    timestamp_ms: int
    imu: IMUReading
    anchors: List[AnchorReading] = field(default_factory=list)
    schema_version: int = SCHEMA_VERSION
    msg_type: str = "telemetry"
    status: Dict[str, Any] = field(default_factory=dict)
    receive_time_unix: float = field(default_factory=time.time)

    def get_anchor(self, anchor_id: str) -> Optional[AnchorReading]:
        for a in self.anchors:
            if a.anchor_id.upper() == anchor_id.upper() or a.ssid.endswith(anchor_id.upper()):
                return a
        return None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "type": self.msg_type,
            "device_id": self.device_id,
            "timestamp_ms": self.timestamp_ms,
            "receive_time_unix": self.receive_time_unix,
            "imu": self.imu.to_dict(),
            "anchors": [a.to_dict() for a in self.anchors],
            "status": self.status,
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict())


@dataclass
class PositionEstimate:
    device_id: str
    timestamp_ms: int
    x_m: float
    y_m: float
    heading_deg: Optional[float]
    method: str
    valid: bool
    uncertainty_m: float
    covariance: Dict[str, float] = field(default_factory=dict)
    anchor_distances: Dict[str, Optional[float]] = field(default_factory=dict)
    raw_rssi: Dict[str, Optional[float]] = field(default_factory=dict)
    server_timestamp: float = field(default_factory=time.time)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "schema_version": SCHEMA_VERSION,
            "type": "position",
            "device_id": self.device_id,
            "timestamp_ms": self.timestamp_ms,
            "server_timestamp": self.server_timestamp,
            "position": {
                "x_m": round(self.x_m, 3) if self.valid else None,
                "y_m": round(self.y_m, 3) if self.valid else None,
                "heading_deg": round(self.heading_deg, 1) if self.heading_deg is not None else None,
                "method": self.method,
                "valid": self.valid,
                "uncertainty_m": round(self.uncertainty_m, 3),
                "covariance": {k: round(v, 4) for k, v in self.covariance.items()},
            },
            "anchor_distances": {
                k: (round(v, 3) if v is not None else None)
                for k, v in self.anchor_distances.items()
            },
            "raw_rssi": self.raw_rssi,
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict())


def parse_telemetry_json(raw_line: str) -> Tuple[Optional[TelemetryMessage], Optional[str]]:
    """
    Parses and validates a newline-delimited JSON line from ESP32 serial/HTTP stream.
    Returns (TelemetryMessage, None) on success, or (None, error_reason) on failure.
    """
    if not raw_line or not raw_line.strip():
        return None, "Empty line"

    raw_line = raw_line.strip()
    # Reject lines that are obviously incomplete or corrupted
    if not (raw_line.startswith("{") and raw_line.endswith("}")):
        return None, "Incomplete or non-JSON envelope"

    try:
        data = json.loads(raw_line)
    except json.JSONDecodeError as e:
        return None, f"JSON decode error: {e}"

    if not isinstance(data, dict):
        return None, "Root payload is not a JSON object"

    device_id = str(data.get("device_id", "tracker_unknown"))
    timestamp_ms = int(data.get("timestamp_ms", int(time.time() * 1000)))

    # Parse IMU section
    imu_raw = data.get("imu", {})
    acc_raw = imu_raw.get("accel_mps2", {})
    gyro_raw = imu_raw.get("gyro_rads", {})

    accel = Vector3(
        x=float(acc_raw.get("x", 0.0)),
        y=float(acc_raw.get("y", 0.0)),
        z=float(acc_raw.get("z", 9.8)),
    )
    gyro = Vector3(
        x=float(gyro_raw.get("x", 0.0)),
        y=float(gyro_raw.get("y", 0.0)),
        z=float(gyro_raw.get("z", 0.0)),
    )
    imu_valid = bool(imu_raw.get("valid", True))
    temp_c = imu_raw.get("temp_c")

    imu = IMUReading(
        accel_mps2=accel,
        gyro_rads=gyro,
        valid=imu_valid,
        temp_c=float(temp_c) if temp_c is not None else None,
    )

    # Parse anchors
    anchors: List[AnchorReading] = []
    raw_anchors = data.get("anchors", [])
    if isinstance(raw_anchors, list):
        for item in raw_anchors:
            if not isinstance(item, dict):
                continue
            anchor_id = str(item.get("anchor_id", item.get("ssid", "?")))
            ssid = str(item.get("ssid", f"INDOOR_ANCHOR_{anchor_id}"))
            rssi_val = item.get("rssi_dbm")

            if rssi_val is None:
                rssi = SENTINEL_RSSI_MISSING
                is_valid = False
            else:
                try:
                    rssi = float(rssi_val)
                    # Check sentinel
                    if rssi <= SENTINEL_RSSI_MISSING or rssi < MIN_PLAUSIBLE_RSSI or rssi > MAX_PLAUSIBLE_RSSI:
                        is_valid = False
                    else:
                        is_valid = bool(item.get("valid", True))
                except (ValueError, TypeError):
                    rssi = SENTINEL_RSSI_MISSING
                    is_valid = False

            age_ms = int(item.get("age_ms", 0))
            anchors.append(
                AnchorReading(
                    anchor_id=anchor_id,
                    ssid=ssid,
                    rssi_dbm=rssi,
                    valid=is_valid,
                    age_ms=age_ms,
                )
            )

    msg = TelemetryMessage(
        device_id=device_id,
        timestamp_ms=timestamp_ms,
        imu=imu,
        anchors=anchors,
        schema_version=int(data.get("schema_version", SCHEMA_VERSION)),
        msg_type=str(data.get("type", "telemetry")),
        status=data.get("status", {}),
    )
    return msg, None
