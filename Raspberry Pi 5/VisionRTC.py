import os
os.environ["QT_QPA_PLATFORM"] = "xcb"

import cv2
import numpy as np
import subprocess
import base64
import threading
import time
import socket
import json
import requests
from inference_sdk import InferenceHTTPClient

# ─── Config ───────────────────────────────────────────────────────────────────
WINDOWS_IP    = "100.82.101.38"
WIDTH, HEIGHT = 1280, 720
INFER_WIDTH   = 640
INFER_HEIGHT  = 360
FPS           = 30
YUV_FRAMESIZE = int(WIDTH * HEIGHT * 1.5)
JPEG_QUALITY  = 75
INFER_INTERVAL = 1.0          # 1 request/sec — safe on Pi
RTSP_BITRATE  = "2M"
UDP_HOST      = "127.0.0.1"
UDP_PORT      = 5005

API_URL = "http://localhost:9001"
API_KEY = "APIKEY" #Insert API Key Here

# ─── Inference client ─────────────────────────────────────────────────────────
client = InferenceHTTPClient(api_url=API_URL, api_key=API_KEY)

# ─── Shared state ─────────────────────────────────────────────────────────────
latest_frame = None
frame_lock   = threading.Lock()
stop_event   = threading.Event()

# ─── UDP socket ───────────────────────────────────────────────────────────────
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_detections(boxes: list):
    try:
        payload = json.dumps({"detections": boxes}).encode()
        udp_sock.sendto(payload, (UDP_HOST, UDP_PORT))
    except Exception as e:
        print(f"[UDP] Send error: {e}")

# ─── Wait for inference server ────────────────────────────────────────────────
def wait_for_server():
    print("[Inference] Waiting for server...", flush=True)
    while True:
        try:
            requests.get(API_URL, timeout=2)
            print("[Inference] Server ready.")
            return
        except Exception:
            time.sleep(3)

# ─── Camera reader ────────────────────────────────────────────────────────────
rpicam_proc = subprocess.Popen(
    ["rpicam-vid", "--camera", "0",
     "--width",  str(WIDTH), "--height", str(HEIGHT),
     "--framerate", str(FPS), "--codec", "yuv420",
     "--timeout", "0", "--nopreview", "-o", "-"],
    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
    bufsize=YUV_FRAMESIZE * 4
)

def camera_reader():
    global latest_frame
    print("[Camera] Reader started.")
    while not stop_event.is_set():
        raw = rpicam_proc.stdout.read(YUV_FRAMESIZE)
        if len(raw) < YUV_FRAMESIZE:
            print("[Camera] Stream ended.")
            stop_event.set()
            break
        yuv = np.frombuffer(raw, dtype=np.uint8).reshape(HEIGHT * 3 // 2, WIDTH)
        bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)
        with frame_lock:
            latest_frame = bgr

# ─── Inference worker ─────────────────────────────────────────────────────────
def inference_worker():
    wait_for_server()
    last_infer = 0
    print("[Inference] Worker started.")
    while not stop_event.is_set():
        elapsed = time.time() - last_infer
        if elapsed < INFER_INTERVAL:
            time.sleep(INFER_INTERVAL - elapsed)
            continue

        with frame_lock:
            frame = latest_frame
        if frame is None:
            time.sleep(0.05)
            continue

        small = cv2.resize(frame, (INFER_WIDTH, INFER_HEIGHT))
        _, jpeg = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        img_b64 = base64.b64encode(jpeg.tobytes()).decode("utf-8")
        last_infer = time.time()

        try:
            result = client.run_workflow(
                workspace_name="yasta-lq2zv",
                workflow_id="detect-count-and-visualize",
                images={"image": f"data:image/jpeg;base64,{img_b64}"},
                use_cache=True,
            )
            raw_result = result[0] if result else {}
            count       = raw_result.get("count_objects")
            preds_wrap  = raw_result.get("predictions", [])

            if isinstance(preds_wrap, dict):
                preds = preds_wrap.get("predictions", [])
            elif isinstance(preds_wrap, list):
                preds = preds_wrap
            else:
                preds = []

            if preds:
                xs = WIDTH  / INFER_WIDTH
                ys = HEIGHT / INFER_HEIGHT
                boxes = [{
                    "x":          int(p["x"] * xs),
                    "y":          int(p["y"] * ys),
                    "width":      int(p["width"]  * xs),
                    "height":     int(p["height"] * ys),
                    "class":      p.get("class", "object"),
                    "confidence": round(p.get("confidence", 0.0), 3),
                } for p in preds]
            else:
                boxes = []

            send_detections(boxes)
            if count is not None:
                print(f"[Inference] Objects: {count}  |  boxes sent via UDP")

        except Exception as e:
            print(f"[Inference] Error: {e}")
            time.sleep(2)           # back-off on error, not 0.5s

# ─── RTSP encoder cam0 ────────────────────────────────────────────────────────
def rtsp_encoder_cam0():
    rtsp_url = f"rtsp://{WINDOWS_IP}:8554/cam0"
    print(f"[RTSP-cam0] Streaming → {rtsp_url}")
    ffmpeg_cmd = [
        "ffmpeg", "-loglevel", "warning",
        "-f", "rawvideo", "-pix_fmt", "bgr24",
        "-s", f"{WIDTH}x{HEIGHT}", "-r", str(FPS), "-i", "pipe:0",
        "-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency",
        "-b:v", RTSP_BITRATE, "-g", str(FPS),
        "-f", "rtsp", "-rtsp_transport", "tcp", rtsp_url,
    ]
    ff = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE, stderr=subprocess.DEVNULL)
    frame_interval = 1.0 / FPS
    next_frame_t   = time.time()
    while not stop_event.is_set():
        now = time.time()
        if now < next_frame_t:
            time.sleep(next_frame_t - now)
        with frame_lock:
            frame = latest_frame
        if frame is None:
            next_frame_t += frame_interval
            continue
        try:
            ff.stdin.write(frame.tobytes())
            ff.stdin.flush()
        except BrokenPipeError:
            print("[RTSP-cam0] ffmpeg pipe broken — restarting...")
            ff.terminate()
            ff = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE, stderr=subprocess.DEVNULL)
        next_frame_t += frame_interval
    ff.stdin.close()
    ff.terminate()

# ─── RTSP cam1 ────────────────────────────────────────────────────────────────
def rtsp_cam1():
    pipe_path = "/tmp/cam1.h264"
    rtsp_url  = f"rtsp://{WINDOWS_IP}:8554/cam1"
    if os.path.exists(pipe_path):
        os.remove(pipe_path)
    os.mkfifo(pipe_path)
    rpicam_cmd = [
        "rpicam-vid", "--camera", "1", "-t", "0", "--nopreview",
        "--codec", "h264", "--width", str(WIDTH), "--height", str(HEIGHT),
        "--framerate", str(FPS), "--inline", "--flush", "-o", pipe_path,
    ]
    ffmpeg_cmd = [
        "ffmpeg", "-loglevel", "warning",
        "-f", "h264", "-r", str(FPS), "-i", pipe_path,
        "-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency",
        "-b:v", RTSP_BITRATE, "-g", str(FPS),
        "-f", "rtsp", "-rtsp_transport", "tcp", rtsp_url,
    ]
    print(f"[RTSP-cam1] Streaming → {rtsp_url}")
    rp = subprocess.Popen(rpicam_cmd, stderr=subprocess.DEVNULL)
    ff = subprocess.Popen(ffmpeg_cmd, stderr=subprocess.DEVNULL)
    ff.wait()
    rp.terminate()

# ─── Launch ───────────────────────────────────────────────────────────────────
threads = [
    threading.Thread(target=camera_reader,      daemon=True, name="CameraReader"),
    threading.Thread(target=inference_worker,   daemon=True, name="InferenceWorker"),
    threading.Thread(target=rtsp_encoder_cam0,  daemon=True, name="RTSP-cam0"),
    threading.Thread(target=rtsp_cam1,          daemon=True, name="RTSP-cam1"),
]
for t in threads:
    t.start()

print("AquaSweep VisionRTC running — Ctrl+C to quit")
try:
    while not stop_event.is_set():
        time.sleep(1)
except KeyboardInterrupt:
    pass
finally:
    stop_event.set()
    rpicam_proc.terminate()
    udp_sock.close()
    if os.path.exists("/tmp/cam1.h264"):
        os.remove("/tmp/cam1.h264")
    print("Stopped.")
