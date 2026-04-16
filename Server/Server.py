"""
AquaSweep — Full Server
Handles:
- WebSocket telemetry relay (Pi → Browser) with cam0 geo-projection
- HLS segment upload/serve (Pi ffmpeg → Browser)
- MediaMTX launcher (WebRTC)
- Serves aquasweep-monitor.html
"""

import asyncio
import json
import math
import os
import subprocess
import threading
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request
from fastapi.responses import HTMLResponse, FileResponse, Response
import uvicorn

app = FastAPI()

# ── HLS DIRECTORIES ───────────────────────────────────────────────────────────
HLS_DIR = "hls"
os.makedirs(f"{HLS_DIR}/cam0", exist_ok=True)
os.makedirs(f"{HLS_DIR}/cam1", exist_ok=True)

# ── CAM0 PROJECTION CONFIG ────────────────────────────────────────────────────
# Cam0: RPi Camera Module 3 Wide — used for inference + geo-projection
# Cam1: video-only, no inference, no projection
CAM0_CONFIG = {
    "hfov_deg":  102.0,   # RPi Camera Module 3 Wide actual HFOV (diagonal is 120°)
    "pitch_deg": -15.0,   # negative = tilted downward toward water
    "height_m":   0.35,   # camera height above water surface in metres
    "img_w":      1280,
    "img_h":      720,
}
R_EARTH = 6371000.0  # metres

def project_box(box, cfg, lat, lon, heading_deg):
    """
    Given a detection box (in full-res pixel coords), camera config,
    and the boat's current GPS + heading, returns a geo dict with:
      lat, lon        — estimated GPS position of the detection
      bearing_offset  — horizontal angle from camera centre axis (degrees)
      dist_m          — estimated distance to object (metres)
    Returns None if the object appears above the horizon.
    """
    fx = cfg["img_w"] / (2.0 * math.tan(math.radians(cfg["hfov_deg"] / 2.0)))
    fy = fx  # square pixels

    cx = cfg["img_w"] / 2.0
    cy = cfg["img_h"] / 2.0

    bx = box["x"] - box["width"]  / 2.0   # box left edge
    by = box["y"] - box["height"] / 2.0   # box top edge
    bw = box["width"]
    bh = box["height"]

    # Horizontal bearing offset from camera centre axis
    bearing_offset = math.degrees(math.atan2((bx + bw / 2.0) - cx, fx))

    # Distance via flat-ground + camera pitch geometry
    # Bottom edge of box = where the object meets the water surface
    bottom_y   = by + bh
    pitch_rad  = math.radians(cfg["pitch_deg"])   # negative = downward tilt
    pixel_down = math.atan2(bottom_y - cy, fy)
    total_down = pitch_rad + pixel_down            # total angle below horizontal

    if total_down >= 0:
        return None  # object is above the horizon — cannot estimate distance

    dist_m = cfg["height_m"] / math.tan(-total_down)

    # Absolute heading to the object in the world frame
    abs_heading = (heading_deg + bearing_offset) % 360
    abs_rad     = math.radians(abs_heading - 90)   # convert to math angle (CCW from east)

    # Haversine offset → GPS coordinate
    d_lat = (dist_m * math.cos(abs_rad)) / R_EARTH
    d_lon = (dist_m * math.sin(abs_rad)) / (
        R_EARTH * math.cos(math.radians(lat))
    )

    return {
        "lat":            lat + math.degrees(d_lat),
        "lon":            lon + math.degrees(d_lon),
        "bearing_offset": round(bearing_offset, 2),
        "dist_m":         round(dist_m, 2),
    }


def enrich_detections(data: dict) -> dict:
    """
    Intercepts the telemetry packet from the Pi.
    For each cam0 detection box, appends a 'geo' field with the
    estimated GPS position calculated from the pinhole camera model.
    Cam1 detections (if any) are passed through untouched.
    If GPS or heading are missing, cam0 boxes pass through without geo.
    """
    gps  = data.get("gps", {})
    imu  = data.get("imu", {})
    dets = data.get("detections", [])

    if not dets:
        return data

    lat = gps.get("lat")
    lon = gps.get("lon")
    hdg = imu.get("heading")

    enriched = []
    for box in dets:
        # Only project cam0 boxes — cam1 passes through as-is
        if box.get("cam", "cam0") != "cam0" or lat is None or lon is None or hdg is None:
            enriched.append(box)
            continue

        geo = project_box(box, CAM0_CONFIG, float(lat), float(lon), float(hdg))
        enriched.append({**box, "geo": geo})  # geo is None if above horizon

    return {**data, "detections": enriched}


# ── WEBSOCKET TELEMETRY ───────────────────────────────────────────────────────
connected_clients: set[WebSocket] = set()
latest_payload: dict = {}

@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    connected_clients.add(ws)
    print(f"Client connected — total: {len(connected_clients)}")

    if latest_payload:
        await ws.send_text(json.dumps(latest_payload))

    try:
        while True:
            raw = await ws.receive_text()
            try:
                data = json.loads(raw)
            except json.JSONDecodeError:
                print(f"Non-JSON message ignored: {raw[:80]}")
                continue

            # Enrich cam0 detections with geo-projection before relaying
            enriched = enrich_detections(data)
            latest_payload.update(enriched)
            out = json.dumps(enriched)

            print(f"Relaying to {len(connected_clients) - 1} browser(s): "
                  f"thr={data.get('uno', {}).get('thr')} "
                  f"heading={data.get('imu', {}).get('heading')}")

            disconnected = set()
            for client in connected_clients:
                if client is ws:
                    continue
                try:
                    await client.send_text(out)
                except Exception:
                    disconnected.add(client)

            for client in disconnected:
                connected_clients.discard(client)

    except WebSocketDisconnect:
        connected_clients.discard(ws)
        print(f"Client disconnected — total: {len(connected_clients)}")


# ── HLS UPLOAD / SERVE ────────────────────────────────────────────────────────
@app.api_route("/hls/{camera}/{filename}", methods=["GET", "PUT", "POST", "DELETE"])
async def hls_handler(camera: str, filename: str, request: Request):
    path = f"{HLS_DIR}/{camera}/{filename}"
    if request.method == "GET":
        if not os.path.exists(path):
            return Response(status_code=404)
        return FileResponse(path)
    elif request.method == "DELETE":
        if os.path.exists(path):
            os.remove(path)
        return Response(status_code=200)
    else:  # PUT, POST
        with open(path, "wb") as f:
            f.write(await request.body())
        return Response(status_code=200)


# ── HTML PAGE ─────────────────────────────────────────────────────────────────
@app.get("/", response_class=HTMLResponse)
async def index():
    with open("templates/aquasweep-monitor.html") as f:
        return f.read()


# ── MEDIAMTX LAUNCHER ────────────────────────────────────────────────────────
def start_mediamtx():
    MEDIAMTX_DIR = "mediamtx"
    subprocess.Popen(
        [f"{MEDIAMTX_DIR}/mediamtx", f"{MEDIAMTX_DIR}/mediamtx.yml"])


# ── ENTRY POINT ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    threading.Thread(target=start_mediamtx, daemon=True).start()
    uvicorn.run(app, host="0.0.0.0", port=8000)
