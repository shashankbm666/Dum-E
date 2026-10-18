"""
Master Runner: Dum-E
-------------------------------------------------------
Runs the Vision Stream Server and optional Mock ESP32 Hardware Simulator concurrently.

Usage:
  python run_server.py                 # Runs Vision Server (port 8000) + ESP32 Mock (port 8080)
  python run_server.py --no-mock       # Runs Vision Server only (connects to real ESP32)
  python run_server.py --synthetic     # Forces synthetic camera simulation
"""

import sys
import os
import time
import argparse
import threading
from src.esp32_mock import create_mock_app
from src.stream_server import app as stream_app

def run_mock_server(port):
    mock_app = create_mock_app()
    mock_app.run(host='0.0.0.0', port=port, debug=False, use_reloader=False)

def main():
    parser = argparse.ArgumentParser(description="Dum-E")
    parser.add_argument("--port", type=int, default=8000, help="Vision server port (default: 8000)")
    parser.add_argument("--mock-port", type=int, default=8080, help="ESP32 mock hardware port (default: 8080)")
    parser.add_argument("--no-mock", action="store_true", help="Disable the local ESP32 mock server")
    parser.add_argument("--synthetic", action="store_true", help="Force synthetic camera pattern")
    parser.add_argument("--esp-ip", type=str, default=None, help="IP/URL of ESP32 (e.g. http://192.168.1.50)")
    args = parser.parse_args()

    if args.synthetic:
        os.environ["USE_SYNTHETIC"] = "true"

    if args.esp_ip:
        os.environ["ESP32_IP"] = args.esp_ip

    print("=" * 64)
    print("   DUM-E")
    print("=" * 64)

    # Start Mock ESP32 server in daemon thread if enabled
    if not args.no_mock:
        mock_thread = threading.Thread(target=run_mock_server, args=(args.mock_port,), daemon=True)
        mock_thread.start()
        time.sleep(0.3)
        print(f" [MOCK ESP32] Running Simulated Hardware on: http://localhost:{args.mock_port}")
        print(f"              API Endpoints: /status, /joint, /grip, /mode")
    def get_local_ip():
        import socket
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(('8.8.8.8', 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return 'localhost'

    local_ip = get_local_ip()

    print(f" [DESKTOP]   Local PC URL:   http://localhost:{args.port}/")
    print(f" [MOBILE]    Phone / Wi-Fi:  http://{local_ip}:{args.port}/")
    if not args.no_mock:
        print(f" [MOCK ESP]  Simulator API:  http://{local_ip}:{args.mock_port}/status")
    print("=" * 64)
    print(" Press Ctrl+C to terminate.")
    print("=" * 64)

    try:
        stream_app.run(host='0.0.0.0', port=args.port, debug=False, use_reloader=False, threaded=True)
    except KeyboardInterrupt:
        print("\n[SYSTEM] Shutting down Dum-E.")

if __name__ == '__main__':
    main()
