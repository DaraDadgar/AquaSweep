# AquaSweep 🌊

An open-source Autonomous Surface Vehicle (ASV) designed for aquatic debris collection. AquaSweep runs on a Raspberry Pi and integrates RC-controlled propulsion, a debris conveyor, dual-camera streaming, AI-powered object detection, and a live browser-based mission monitor — all connected over a Tailscale VPN tunnel.

---

## Table of Contents

- [Architecture Overview](#architecture-overview)
- [Hardware](#hardware)
- [File Structure](#file-structure)
- [Raspberry Pi Setup](#raspberry-pi-setup)
- [Arduino Uno Setup](#arduino-uno-setup)
- [Windows Server Setup](#windows-server-setup)
- [Running AquaSweep](#running-aquasweep)
- [Mission Monitor](#mission-monitor)
- [Channel Mapping (FlySky iBUS)](#channel-mapping-flysky-ibus)
- [Telemetry Packet Format](#telemetry-packet-format)
- [Debug Mode (Arduino)](#debug-mode-arduino)

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────┐
│                    Raspberry Pi                          │
│                                                          │
│  shared_state.py  (orchestrator)                         │
│      ├── VisionRTC.py                                    │
│      │     ├── rpicam-vid (cam0) → YUV → BGR             │
│      │     ├── Roboflow Inference (localhost:9001)       │
│      │     ├── ffmpeg → RTSP/cam0 → MediaMTX            │
│      │     ├── rpicam-vid (cam1) → H264 → RTSP/cam1     │
│      │     └── UDP detections → localhost:5005           │
│      │                                                   │
│      └── telemetry.py                                    │
│            ├── Arduino Uno (USB Serial, 115200)          │
│            ├── GPS Module (/dev/serial0, 9600)           │
│            ├── BNO055 IMU (I2C)                          │
│            ├── UDP listener ← localhost:5005             │
│            └── WebSocket → ws://[VPS]:8000/ws            │
└─────────────────────────────────────────────────────────┘
                          │ Tailscale VPN
┌─────────────────────────────────────────────────────────┐
│                  Windows Server (VPS)                    │
│                                                          │
│  Server.py  (FastAPI)                                    │
│      ├── /ws  WebSocket relay (Pi → Browser)            │
│      └── /    Serves aquasweep_monitor.html             │
│                                                          │
│  MediaMTX                                               │
│      ├── :8554  RTSP input from Pi                      │
│      └── :8889  WebRTC output to Browser                │
└─────────────────────────────────────────────────────────┘
                          │ Browser
┌─────────────────────────────────────────────────────────┐
│              aquasweep_monitor.html                      │
│   Live telemetry · AI overlay · GPS map · Dual-camera   │
└─────────────────────────────────────────────────────────┘
```

---

## Hardware

| Component | Details |
|---|---|
| Main computer | Raspberry Pi (Camera-capable, tested on Pi 5) |
| Microcontroller | Arduino Uno |
| RC Receiver | FlySky iBUS-compatible receiver |
| Thrusters | 2× brushless ESCs (PWM, 1000–2000 µs) |
| Conveyor motor | DC motor via IBT-2 / BTS7960 H-bridge driver |
| Overflow detection | 2× JSN-SR04T waterproof ultrasonic sensors |
| IMU | Adafruit BNO055 (I2C) |
| GPS | UART GPS module on `/dev/serial0` |
| Cameras | 2× Raspberry Pi CSI cameras (wide and standard) |

---

## File Structure

```
AquaSweep/
├── manual_operation.ino      # Arduino Uno firmware (RC control + telemetry output)
├── shared_state.py           # Pi orchestrator — launches and monitors all scripts
├── VisionRTC.py              # Camera capture, Roboflow inference, RTSP streaming
├── telemetry.py              # Sensor reader + WebSocket sender + UDP detection listener
├── Server.py                 # FastAPI WebSocket relay + MediaMTX launcher (Windows)
├── aquasweep_monitor.html    # Browser dashboard (telemetry, map, video, AI overlay)
└── Dependencies.txt          # Full dependency and install reference
```

---

## Raspberry Pi Setup

### 1. System Packages

```bash
sudo apt update
sudo apt install -y ffmpeg libcamera-apps python3-venv python3-pip
```

### 2. Install pyenv Dependencies

```bash
sudo apt install -y make build-essential libssl-dev zlib1g-dev \
libbz2-dev libreadline-dev libsqlite3-dev wget curl llvm \
libncursesw5-dev xz-utils tk-dev libxml2-dev libxmlsec1-dev \
libffi-dev liblzma-dev
```

### 3. Install pyenv and Python 3.11.9

```bash
curl https://pyenv.run | bash

# Add pyenv to your shell
echo 'export PYENV_ROOT="$HOME/.pyenv"' >> ~/.bashrc
echo 'export PATH="$PYENV_ROOT/bin:$PATH"' >> ~/.bashrc
echo 'eval "$(pyenv init -)"' >> ~/.bashrc
source ~/.bashrc

pyenv install 3.11.9
```

### 4. Create the Virtual Environment

```bash
cd ~/AquaSweep
pyenv local 3.11.9
python -m venv .venv --system-site-packages
source .venv/bin/activate
```

### 5. Install Python Dependencies

```bash
# Core dependencies
pip install -U numpy opencv-python websockets pyserial pynmea2 \
    adafruit-blinka adafruit-circuitpython-bno055

# Computer vision / Roboflow
pip install -U inference-sdk
pip install -U inference-cli
```

### 6. Install Docker (required for Roboflow Inference Server)

```bash
# Remove old Docker installations
sudo apt remove -y docker docker-engine docker.io containerd runc

# Install Docker
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker $USER
newgrp docker
```

### 7. Install and Configure Tailscale

```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
sudo systemctl enable tailscaled
```

Note the Pi's Tailscale IP — you will need it in `telemetry.py` (`WS_URI`) and `VisionRTC.py` (`WINDOWS_IP`).

### 8. Enable Camera and Serial Interfaces

Using `raspi-config`:
- Enable **Camera** (Legacy or Libcamera depending on Pi OS version)
- Enable **I2C** (for BNO055)
- Enable **Serial Port** / disable serial console (for GPS on `/dev/serial0`)

---

## Arduino Uno Setup

### Wiring Reference

| Function | Arduino Pin |
|---|---|
| Left ESC (PWM) | D9 |
| Right ESC (PWM) | D3 |
| Conveyor RPWM | D5 |
| Conveyor LPWM | D11 |
| Conveyor REN | D2 |
| Conveyor LEN | D13 |
| Ultrasonic 1 TRIG | D6 |
| Ultrasonic 1 ECHO | D7 |
| Ultrasonic 2 TRIG | D4 |
| Ultrasonic 2 ECHO | A1 |
| iBUS RC Input | D0 (RX, hardware Serial) |
| Telemetry Output | D1 (TX → Pi USB `/dev/ttyACM0`) |

> ⚠️ **Important:** In RUN mode, the iBUS RC signal shares hardware Serial (D0) with the USB connection. Disconnect the USB cable from the Uno before plugging in the RC receiver in RUN mode, or use DEBUG mode during development (see below).

### Required Arduino Library

Install via Arduino IDE Library Manager:
- **Servo** (built-in)
- **SoftwareSerial** (built-in, used in DEBUG mode only)

### Flashing

1. Open `manual_operation.ino` in the Arduino IDE.
2. Set `DEBUG_MODE` at the top of the file:
   - `0` → **RUN mode**: iBUS on D0, JSON telemetry output on USB Serial at 10 Hz. Use this for normal operation.
   - `1` → **DEBUG mode**: iBUS on D8 (SoftwareSerial), Serial Monitor prints at 5 Hz, motors held neutral. Use this for bench testing.
3. Select **Arduino Uno** as board and the correct COM port.
4. Upload.

### ESC Arming

On first flash or after a long power-off, the ESCs may need to be armed. The sketch sends 1500 µs (neutral) to both ESCs for 7 seconds on startup to allow the ESC arming sequence to complete. Make sure the boat is in a safe position during this window.

### Overflow Detection Tuning

The two JSN-SR04T sensors use a confidence-based hysteresis system to avoid false positives from splashing water. Adjust these constants in `manual_operation.ino` if needed:

```cpp
float FULL_ON_CM  = 45.0;   // Distance (cm) at which bin is considered full
float FULL_OFF_CM = 55.0;   // Distance (cm) at which full flag clears
const uint8_t FULL_CONF_COUNT = 10; // Consecutive readings required to change state
```

---

## Windows Server Setup

### 1. Install Python Dependencies

```bash
pip install fastapi uvicorn websockets
```

### 2. Download MediaMTX

Download the latest MediaMTX release for Windows from [https://github.com/bluenviron/mediamtx/releases](https://github.com/bluenviron/mediamtx/releases) and extract it to a folder named:

```
mediamtx_v1.17.0_windows_amd64\
```

Place this folder in the same directory as `Server.py`.

### 3. Place the Monitor HTML

```
your-server-folder/
├── Server.py
├── mediamtx_v1.17.0_windows_amd64/
└── templates/
    └── aquasweep_monitor.html
```

### 4. Update IP Addresses

In `aquasweep_monitor.html`, update the following constants to match your Tailscale IPs:

```javascript
const WS_URI = "ws://YOUR_VPS_TAILSCALE_IP:8000/ws";
const MEDIAMTX_URL = "http://YOUR_VPS_TAILSCALE_IP:8889";
```

In `telemetry.py` and `VisionRTC.py` on the Pi, update:

```python
WS_URI     = "ws://YOUR_VPS_TAILSCALE_IP:8000/ws"  # telemetry.py
WINDOWS_IP = "YOUR_VPS_TAILSCALE_IP"               # VisionRTC.py
```

---

## Running AquaSweep

### Start the Roboflow Inference Server (Pi)

```bash
inference server start
```

Wait for the Docker container to be ready before launching the main scripts.

### Start the Server (Windows)

```bash
python Server.py
```

This launches FastAPI on port `8000` and MediaMTX on ports `8554` (RTSP) and `8889` (WebRTC) automatically.

### Start the Pi Scripts

```bash
cd ~/AquaSweep
source .venv/bin/activate
python shared_state.py
```

`shared_state.py` launches both `VisionRTC.py` and `telemetry.py` as subprocesses and automatically restarts either one if it crashes.

### Open the Monitor

Navigate to `http://YOUR_VPS_TAILSCALE_IP:8000` in any browser on the Tailscale network.

---

## Mission Monitor

`aquasweep_monitor.html` provides a real-time dashboard including:

- **Dual camera feeds** via WebRTC with picture-in-picture and one-click swap
- **AI detection overlay** — toggle bounding boxes from the Roboflow model on the live feed
- **Telemetry panel** — throttle, steering, heading (compass), conveyor state, E-stop, overflow, RC signal, GPS fix
- **GPS track map** — Google Maps minimap with live boat position, heading cone, and breadcrumb trail
- **Missing data alerts** — banner that lists every sensor field that is null or stale
- **Simulation mode** — auto-activates with realistic fake data when the WebSocket is disconnected, so the UI is always testable

---

## Channel Mapping (FlySky iBUS)

| Channel | Index | Function |
|---|---|---|
| CH3 | 2 | Throttle |
| CH4 | 3 | Steering |
| CH5 | 4 | E-Stop (toggle) |
| CH6 | 5 | Conveyor Speed (knob) |
| CH7 | 6 | Conveyor Direction (toggle) |
| CH8 | 7 | Conveyor On/Off (toggle) |

---

## Telemetry Packet Format

`telemetry.py` sends the following JSON structure over WebSocket at 5 Hz:

```json
{
  "ts": 1713000000.0,
  "uno": {
    "thr": 55,
    "str": 50,
    "conv": 1,
    "conv_dir": 0,
    "conv_speed": 180,
    "estop": 0,
    "overflow": 0,
    "rc_lost": 0,
    "ts": 1713000000.0
  },
  "gps": {
    "lat": 45.5017,
    "lon": -73.5673,
    "fix": 3,
    "sats": 9
  },
  "imu": {
    "heading": 182.4
  },
  "detections": [
    {
      "x": 640, "y": 360,
      "width": 120, "height": 80,
      "class": "bottle",
      "confidence": 0.87
    }
  ]
}
```

---

## Debug Mode (Arduino)

Set `#define DEBUG_MODE 1` at the top of `manual_operation.ino` before flashing to enable:

- iBUS receiver connected to **D8** (SoftwareSerial) instead of D0 — safe to use with USB connected
- Motors held at neutral, conveyor stopped — nothing moves on the bench
- Serial Monitor output at **5 Hz** showing all channel values, ultrasonic readings, overflow state, and RC lost flag
- Baud rate: **115200**

Switch back to `DEBUG_MODE 0` before deploying on the water.

---

## License

MIT
