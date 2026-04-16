"""
AquaSweep — Orchestrator
=========================
Launches and monitors all Pi-side scripts.
Detection state is no longer stored here — VisionRTC.py sends detections
directly to telemetry_reader_senderV4.py via UDP (localhost:5005).

Scripts:
  VisionRTC.py                  — camera + inference + RTSP streams
  telemetry_reader_senderV4.py  — sensors + WebSocket sender + UDP listener
"""

import subprocess
import sys
import os
import signal
import time

SCRIPTS = {
    "VisionRTC":  "VisionRTC.py",
    "Telemetry":  "telemetry_reader_senderV4.py",
}

def start():
    python = sys.executable
    processes = {
        name: subprocess.Popen([python, script])
        for name, script in SCRIPTS.items()
    }

    print("AquaSweep started:")
    for name, p in processes.items():
        print(f"  {name} → PID {p.pid}")

    def shutdown(sig=None, frame=None):
        print("\nShutting down AquaSweep...")
        for name, p in processes.items():
            p.terminate()
            print(f"  {name} stopped.")
        sys.exit(0)

    signal.signal(signal.SIGINT,  shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    while True:
        for name, script in SCRIPTS.items():
            p = processes[name]
            if p.poll() is not None:
                print(f"[Orchestrator] {name} crashed (exit {p.returncode}) — restarting...")
                processes[name] = subprocess.Popen([python, script])
        time.sleep(5)

if __name__ == "__main__":
    start()
