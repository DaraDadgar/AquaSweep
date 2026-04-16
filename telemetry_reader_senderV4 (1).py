"""
AquaSweep — Pi Telemetry Bridge (V4)
======================================
Key change vs V3:
  - Detections no longer come from shared_state (which broke cross-process).
  - A background UDP listener thread receives JSON from VisionRTC.py on
    localhost:5005 and stores the latest boxes in a local variable.
  - Everything else (sensors, WebSocket sender) is unchanged.
"""

import asyncio
import json
import time
import threading
import socket
import websockets

# =============================================================================
# CONFIG
# =============================================================================

WS_URI             = "ws://100.82.101.38:8000/ws"
UNO_PORT           = "/dev/ttyACM0"
UNO_BAUD           = 115200
GPS_PORT           = "/dev/serial0"
GPS_BAUD           = 9600
PUBLISH_HZ         = 5.0
PUBLISH_PERIOD     = 1.0 / PUBLISH_HZ
WS_RETRY_DELAY     = 5.0
SENSOR_RETRY_DELAY = 10.0

UDP_HOST           = "127.0.0.1"
UDP_PORT           = 5005

# =============================================================================
# DETECTION STATE  (filled by UDP listener thread)
# =============================================================================

_det_lock      = threading.Lock()
_latest_boxes  = []

def _udp_listener():
    global _latest_boxes
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_HOST, UDP_PORT))
    sock.settimeout(1.0)
    print(f"[UDP] Listening for detections on {UDP_HOST}:{UDP_PORT}")
    while True:
        try:
            data, _ = sock.recvfrom(65535)
            obj = json.loads(data.decode())
            boxes = obj.get("detections", [])
            with _det_lock:
                _latest_boxes = boxes
        except socket.timeout:
            continue
        except Exception as e:
            print(f"[UDP] Parse error: {e}")

threading.Thread(target=_udp_listener, daemon=True, name="UDPListener").start()

# =============================================================================
# HARDWARE HANDLES
# =============================================================================

uno_ser    = None
gps_ser    = None
imu_sensor = None

# =============================================================================
# SHARED SENSOR STATE
# =============================================================================

latest_uno = {
    "thr": None, "str": None,
    "conv": None, "conv_dir": None,
    "conv_actual": None, "conv_actual_dir": None,
    "estop": None, "overflow": None, "rc_lost": None, "ts": None,
}
latest_gps = {"lat": None, "lon": None, "fix": 0, "sats": 0}
latest_imu = {"heading": None}

# =============================================================================
# HELPERS
# =============================================================================

def safe_float(value):
    try:    return float(value)
    except: return None

def safe_int(value, default=0):
    try:    return int(value)
    except: return default

# =============================================================================
# HARDWARE INIT
# =============================================================================

def init_uno():
    try:
        import serial
        ser = serial.Serial(UNO_PORT, UNO_BAUD, timeout=0.05)
        print(f"  [UNO] ✓ Connected on {UNO_PORT}")
        return ser
    except Exception as e:
        print(f"  [UNO] ✗ Not available ({e})")
        return None

def init_gps():
    try:
        import serial
        ser = serial.Serial(GPS_PORT, GPS_BAUD, timeout=0.05)
        print(f"  [GPS] ✓ Connected on {GPS_PORT}")
        return ser
    except Exception as e:
        print(f"  [GPS] ✗ Not available ({e})")
        return None

def init_imu():
    try:
        import board, busio, adafruit_bno055
        i2c = busio.I2C(board.SCL, board.SDA)
        try:
            sensor = adafruit_bno055.BNO055_I2C(i2c, address=0x29)
            print("  [IMU] ✓ Connected at I2C 0x29")
        except Exception:
            sensor = adafruit_bno055.BNO055_I2C(i2c, address=0x28)
            print("  [IMU] ✓ Connected at I2C 0x28")
        return sensor
    except Exception as e:
        print(f"  [IMU] ✗ Not available ({e})")
        return None

# =============================================================================
# SENSOR READERS
# =============================================================================

def read_uno():
    global uno_ser
    if uno_ser is None:
        return
    try:
        import serial
        line = uno_ser.readline().decode("utf-8", errors="replace").strip()
        if not line:
            return
        try:
            data = json.loads(line)
            if isinstance(data, dict):
                latest_uno.update(data)
        except json.JSONDecodeError:
            pass
    except Exception:
        print("[UNO] Lost connection — will retry...")
        uno_ser = None

def read_gps():
    global gps_ser
    if gps_ser is None:
        return
    try:
        import pynmea2
        line = gps_ser.readline().decode("ascii", errors="replace").strip()
        if not line:
            return
        try:
            if line.startswith(("$GNGGA", "$GPGGA")):
                msg = pynmea2.parse(line)
                latest_gps["lat"]  = safe_float(msg.latitude)  if msg.latitude  else None
                latest_gps["lon"]  = safe_float(msg.longitude) if msg.longitude else None
                latest_gps["fix"]  = safe_int(msg.gps_qual, 0)
                latest_gps["sats"] = safe_int(msg.num_sats, 0)
            elif line.startswith(("$GNRMC", "$GPRMC")):
                msg = pynmea2.parse(line)
                if getattr(msg, "status", "") == "A":
                    latest_gps["lat"] = safe_float(msg.latitude)  or latest_gps["lat"]
                    latest_gps["lon"] = safe_float(msg.longitude) or latest_gps["lon"]
        except Exception:
            pass
    except Exception:
        print("[GPS] Lost connection — will retry...")
        gps_ser = None

def read_imu():
    global imu_sensor
    if imu_sensor is None:
        return
    try:
        euler = imu_sensor.euler
        if euler is not None and euler[0] is not None:
            latest_imu["heading"] = round(float(euler[0]), 2)
    except Exception:
        print("[IMU] Lost connection — will retry...")
        imu_sensor = None

# =============================================================================
# PACKET BUILDER
# =============================================================================

def build_packet():
    with _det_lock:
        boxes = list(_latest_boxes)
    return {
        "ts":         time.time(),
        "uno":        latest_uno.copy(),
        "gps":        latest_gps.copy(),
        "imu":        latest_imu.copy(),
        "detections": boxes,
    }

# =============================================================================
# ASYNC LOOPS
# =============================================================================

async def sensor_retry_loop():
    global uno_ser, gps_ser, imu_sensor
    while True:
        await asyncio.sleep(SENSOR_RETRY_DELAY)
        if uno_ser    is None: uno_ser    = init_uno()
        if gps_ser    is None: gps_ser    = init_gps()
        if imu_sensor is None: imu_sensor = init_imu()

async def sensor_loop():
    print("Sensor loop started.")
    while True:
        read_uno()
        read_gps()
        read_imu()
        await asyncio.sleep(0)

async def ws_sender():
    while True:
        try:
            print(f"Connecting to {WS_URI} ...")
            async with websockets.connect(WS_URI) as ws:
                print(f"Connected. Broadcasting at {PUBLISH_HZ} Hz.\n")
                next_publish = time.time()
                while True:
                    now = time.time()
                    if now >= next_publish:
                        packet = build_packet()
                        await ws.send(json.dumps(packet))

                        uno  = packet["uno"]
                        gps  = packet["gps"]
                        imu  = packet["imu"]
                        dets = packet["detections"]

                        det_str = (
                            ", ".join(
                                f"{d.get('class','?')}({d.get('confidence',0):.0%})"
                                for d in dets
                            )
                            if dets else "none"
                        )

                        print(
                            f"[{time.strftime('%H:%M:%S')}]  "
                            f"thr={uno.get('thr')}%  "
                            f"str={uno.get('str')}%  "
                            f"hdg={imu.get('heading')}°  "
                            f"gps=({gps.get('lat')}, {gps.get('lon')})  "
                            f"fix={gps.get('fix')}  "
                            f"estop={uno.get('estop')}  "
                            f"overflow={uno.get('overflow')}  "
                            f"rc_lost={uno.get('rc_lost')}  "
                            f"detections=[{det_str}]"
                        )
                        next_publish = now + PUBLISH_PERIOD
                    await asyncio.sleep(0.01)

        except (OSError, websockets.exceptions.InvalidURI) as e:
            print(f"Could not connect: {e}. Retrying in {WS_RETRY_DELAY}s...")
            await asyncio.sleep(WS_RETRY_DELAY)
        except websockets.exceptions.ConnectionClosedError as e:
            print(f"Connection dropped: {e}. Reconnecting in {WS_RETRY_DELAY}s...")
            await asyncio.sleep(WS_RETRY_DELAY)

# =============================================================================
# ENTRY POINT
# =============================================================================

async def main():
    global uno_ser, gps_ser, imu_sensor

    print("AquaSweep Pi Bridge V4 starting...")
    print(f"  WS   → {WS_URI}")
    print(f"  Rate → {PUBLISH_HZ} Hz")
    print("Initializing hardware...\n")

    uno_ser    = init_uno()
    gps_ser    = init_gps()
    imu_sensor = init_imu()

    print(f"\nHardware status:")
    print(f"  UNO → {'OK' if uno_ser    else 'MISSING — will retry'}")
    print(f"  GPS → {'OK' if gps_ser    else 'MISSING — will retry'}")
    print(f"  IMU → {'OK' if imu_sensor else 'MISSING — will retry'}")
    print()

    try:
        await asyncio.gather(
            sensor_loop(),
            sensor_retry_loop(),
            ws_sender(),
        )
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        if uno_ser:  uno_ser.close()
        if gps_ser:  gps_ser.close()
        print("Serial ports closed.")

if __name__ == "__main__":
    asyncio.run(main())
