"""
Full-Stack Web Server and API Gateway for ESP32 Indoor Positioning.

Built with Python's high-performance standard library ThreadingHTTPServer.
Features:
- Static file delivery (HTML, CSS, JS)
- REST APIs (/api/status, /api/history, /api/events, /api/config, /api/telemetry)
- Real-time Server-Sent Events (SSE) streaming at /api/live for 10Hz dashboard updates
- Dual ingestion: receives data from SerialIngestionService or directly via HTTP POST
"""

import json
import mimetypes
import os
import queue
import threading
import time
from http.server import HTTPServer, SimpleHTTPRequestHandler
from socketserver import ThreadingMixIn
from typing import Dict, List, Optional, Any
from urllib.parse import urlparse, parse_qs

from core.protocol import TelemetryMessage, PositionEstimate, parse_telemetry_json
from core.position_estimator import PositionEstimator
from core.storage import StorageManager
from core.serial_ingestion import SerialIngestionService


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Multi-threaded HTTP Server handling concurrent REST and SSE connections."""
    daemon_threads = True
    allow_reuse_address = True


class SystemState:
    """Thread-safe global state holder shared across server, ingestion, and estimator."""

    def __init__(self, config: Dict[str, Any], static_dir: str):
        self.config = config
        self.static_dir = static_dir
        self.storage = StorageManager(
            db_path=config.get("storage", {}).get("db_path", "indoor_positioning.db"),
            max_records=config.get("storage", {}).get("max_records", 50000),
        )
        self.estimator = PositionEstimator(config)
        self.latest_telemetry: Optional[TelemetryMessage] = None
        self.latest_position: Optional[PositionEstimate] = None
        self.lock = threading.Lock()

        # SSE subscriber queues
        self.sse_subscribers: List[queue.Queue] = []
        self.subscribers_lock = threading.Lock()

        # Serial Ingestion Service
        self.serial_service = SerialIngestionService(
            config=config,
            on_telemetry_callback=self.on_telemetry_received,
            on_event_callback=self.on_event_received,
        )

    def on_telemetry_received(self, msg: TelemetryMessage):
        """Processes incoming telemetry from serial or HTTP."""
        with self.lock:
            self.latest_telemetry = msg
            # Position Estimation
            est = self.estimator.process_telemetry(msg)
            self.latest_position = est

        # Persist
        try:
            self.storage.log_telemetry(msg)
            self.storage.log_position(est)
        except Exception as e:
            print(f"[STORAGE ERROR] {e}")

        # Broadcast to SSE clients
        payload = json.dumps({
            "type": "update",
            "telemetry": msg.to_dict(),
            "position": est.to_dict(),
            "server_time": time.time(),
        })
        self.broadcast_sse(payload)

    def on_event_received(self, event_type: str, severity: str, message: str):
        self.storage.log_event("gateway", event_type, severity, message)
        payload = json.dumps({
            "type": "event",
            "event_type": event_type,
            "severity": severity,
            "message": message,
            "timestamp": time.time(),
        })
        self.broadcast_sse(payload)

    def add_subscriber(self) -> queue.Queue:
        q = queue.Queue(maxsize=50)
        with self.subscribers_lock:
            self.sse_subscribers.append(q)
        return q

    def remove_subscriber(self, q: queue.Queue):
        with self.subscribers_lock:
            if q in self.sse_subscribers:
                self.sse_subscribers.remove(q)

    def broadcast_sse(self, data: str):
        with self.subscribers_lock:
            dead = []
            for q in self.sse_subscribers:
                try:
                    q.put_nowait(data)
                except queue.Full:
                    # Client buffer full; drop to prevent lag
                    pass
                except Exception:
                    dead.append(q)
            for d in dead:
                if d in self.sse_subscribers:
                    self.sse_subscribers.remove(d)


def make_request_handler(state: SystemState):
    class AppRequestHandler(SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            self.state = state
            super().__init__(*args, directory=state.static_dir, **kwargs)

        def log_message(self, format, *args):
            # Suppress default noisy console access logging
            pass

        def send_json(self, status_code: int, data: Any):
            body = json.dumps(data).encode("utf-8")
            self.send_response(status_code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.end_headers()
            self.wfile.write(body)

        def do_OPTIONS(self):
            self.send_response(200)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.end_headers()

        def do_GET(self):
            parsed = urlparse(self.path)
            path = parsed.path
            params = parse_qs(parsed.query)

            if path == "/api/status":
                with self.state.lock:
                    pos = self.state.latest_position.to_dict() if self.state.latest_position else None
                    tel = self.state.latest_telemetry.to_dict() if self.state.latest_telemetry else None
                    connected = self.state.serial_service.is_connected
                    simulated = self.state.serial_service.is_simulated

                self.send_json(200, {
                    "connected": connected,
                    "simulated": simulated,
                    "position": pos,
                    "telemetry": tel,
                    "server_time": time.time(),
                })
                return

            elif path == "/api/config":
                self.send_json(200, self.state.config)
                return

            elif path == "/api/history":
                limit = int(params.get("limit", [200])[0])
                dev_id = params.get("device_id", [None])[0]
                history = self.state.storage.query_positions(device_id=dev_id, limit=limit)
                self.send_json(200, history)
                return

            elif path == "/api/events":
                limit = int(params.get("limit", [50])[0])
                events = self.state.storage.query_events(limit=limit)
                self.send_json(200, events)
                return

            elif path == "/api/live":
                # Server-Sent Events (SSE) Stream
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Connection", "keep-alive")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()

                q = self.state.add_subscriber()
                try:
                    # Send initial handshake
                    self.wfile.write(b"data: {\"type\":\"connected\"}\n\n")
                    self.wfile.flush()

                    while True:
                        try:
                            msg = q.get(timeout=2.0)
                            out = f"data: {msg}\n\n".encode("utf-8")
                            self.wfile.write(out)
                            self.wfile.flush()
                        except queue.Empty:
                            # Keep-alive heartbeat comment
                            self.wfile.write(b": keep-alive\n\n")
                            self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    pass
                finally:
                    self.state.remove_subscriber(q)
                return

            # Default static file routing
            super().do_GET()

        def do_POST(self):
            parsed = urlparse(self.path)
            path = parsed.path

            if path == "/api/telemetry":
                # Ingestion endpoint for Wi-Fi or Cloud Forwarding
                content_len = int(self.headers.get("Content-Length", 0))
                raw_body = self.rfile.read(content_len).decode("utf-8", errors="replace")
                msg, err = parse_telemetry_json(raw_body)
                if msg:
                    self.state.on_telemetry_received(msg)
                    self.send_json(200, {"status": "ok", "timestamp_ms": msg.timestamp_ms})
                else:
                    self.send_json(400, {"status": "error", "reason": err})
                return

            self.send_json(404, {"error": "Not found"})

    return AppRequestHandler
