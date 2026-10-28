"""
Unified Entrypoint and System Launcher for ESP32 Indoor Positioning System.

Starts:
1. Configuration loader (config.json)
2. Storage manager (SQLite persistence)
3. Position estimator (Adaptive Kalman filter)
4. Hardware serial ingestion (with simulated hardware fallback)
5. Web dashboard and API server (http://127.0.0.1:8080)
"""

import json
import os
import signal
import sys
import time

from web.server import ThreadedHTTPServer, SystemState, make_request_handler


def load_config(config_path: str = "config.json") -> dict:
    if not os.path.exists(config_path):
        print(f"[ERROR] Config file '{config_path}' not found!")
        sys.exit(1)
    with open(config_path, "r", encoding="utf-8") as f:
        return json.load(f)


def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(base_dir, "config.json")
    static_dir = os.path.join(base_dir, "web", "static")

    config = load_config(config_path)
    server_cfg = config.get("server", {})
    host = server_cfg.get("host", "127.0.0.1")
    port = int(server_cfg.get("port", 8080))

    print("=" * 64)
    print("   ESP32 INDOOR POSITIONING HARDWARE INTEGRATION SUITE")
    print("=" * 64)
    print(f"[*] Base Directory: {base_dir}")
    print(f"[*] Static Assets:  {static_dir}")
    print(f"[*] Target Serial:  {config.get('serial', {}).get('port', 'COM3')} @ {config.get('serial', {}).get('baud_rate', 115200)} baud")
    print(f"[*] Anchors Config: {len(config.get('anchors', []))} fixed beacons")

    for a in config.get("anchors", []):
        print(f"    - Anchor {a.get('id')}: SSID '{a.get('ssid')}' at coords {a.get('coordinates')}")

    # Initialize shared system state
    state = SystemState(config, static_dir)

    # Start Serial Ingestion Layer
    print("\n[+] Starting Serial Ingestion Layer...")
    state.serial_service.start()

    # Start HTTP Server
    handler_class = make_request_handler(state)
    httpd = ThreadedHTTPServer((host, port), handler_class)

    print(f"\n[+] Live Dashboard running at: http://{host}:{port}/")
    print("[+] REST API Endpoints:")
    print(f"    - Status:     http://{host}:{port}/api/status")
    print(f"    - Live SSE:   http://{host}:{port}/api/live")
    print(f"    - History:    http://{host}:{port}/api/history?limit=100")
    print(f"    - Config:     http://{host}:{port}/api/config")
    print(f"    - Ingestion:  POST http://{host}:{port}/api/telemetry")
    print("=" * 64)
    print("Press Ctrl+C to terminate cleanly.\n")

    def handle_shutdown(signum, frame):
        print("\n[*] Shutting down ESP32 Indoor Positioning System...")
        state.serial_service.stop()
        httpd.shutdown()
        httpd.server_close()
        print("[*] Shutdown complete.")
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_shutdown)
    signal.signal(signal.SIGTERM, handle_shutdown)

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        handle_shutdown(None, None)


if __name__ == "__main__":
    main()
