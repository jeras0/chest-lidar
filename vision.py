"""Webcam object detection fused with the chest lidar, with spoken alerts.

  - runs YOLO on the USB webcam (TensorRT on the Orin)
  - reads the latest scan from the running obstacle_alert.py and measures how far away each object is
  - speaks e.g. "person ahead, 2 meters" through the default speaker (the Bluetooth Nest Mini)
  - serves the labelled camera feed and detections on port 8001; viewer.html shows them

Start obstacle_alert.py first, then run this as your normal user (not sudo, so Bluetooth audio works):
    python3 vision.py
Test without a camera or YOLO:  python3 vision.py --sim   (next to obstacle_alert.py --sim)
"""
import argparse
import json
import math
import os
import shutil
import subprocess
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

from lidar_scip import step_to_angle

HERE = os.path.dirname(os.path.abspath(__file__))

# COCO classes worth telling someone who is walking about. Everything else is ignored.
ANNOUNCE = {
    "person", "bicycle", "car", "motorcycle", "bus", "truck", "dog", "cat",
    "chair", "bench", "couch", "dining table", "bed", "toilet", "potted plant",
    "fire hydrant", "stop sign", "suitcase", "backpack", "refrigerator", "tv", "laptop",
}
SPOKEN_NAME = {"dining table": "table", "tv": "screen", "potted plant": "plant"}


class LidarFeed:
    """Follows the /stream of obstacle_alert.py and keeps the latest scan."""

    def __init__(self, url):
        self.url = url
        self.scan = ([], 1, 0.0)  # (ranges in mm, step between readings, time received)
        threading.Thread(target=self.run, daemon=True).start()

    def run(self):
        while True:
            try:
                with urllib.request.urlopen(self.url, timeout=5) as resp:
                    print("lidar feed connected:", self.url)
                    for line in resp:
                        if line.startswith(b"data: "):
                            f = json.loads(line[6:])
                            self.scan = (f["r"], f["step"], time.time())
            except Exception as e:
                print("lidar feed: %s - retrying (is obstacle_alert.py running?)" % e)
                time.sleep(2)

    def distance(self, a_left, a_right):
        """Distance (m) to what the lidar sees between two angles (radians, left positive),
        or None when nothing is in that slice or the feed is stale."""
        ranges, step, t = self.scan
        if time.time() - t > 1.0:
            return None
        vals = sorted(
            mm for k, mm in enumerate(ranges)
            if mm >= 20 and a_right <= step_to_angle(k * step) <= a_left
        )
        if not vals:
            return None
        return vals[len(vals) // 4] / 1000.0  # lower quartile: the object, not the wall behind it


def pixel_to_angle(x, width, hfov_deg, offset_deg):
    """Image column -> lidar angle in radians (0 = straight ahead, positive = left)."""
    f = (width / 2) / math.tan(math.radians(hfov_deg / 2))
    return math.atan((width / 2 - x) / f) + math.radians(offset_deg)


class Detector:
    def __init__(self, model, conf, imgsz):
        from ultralytics import YOLO
        self.model = YOLO(model, task="detect")
        self.conf, self.imgsz = conf, imgsz

    def __call__(self, img):
        r = self.model(img, conf=self.conf, imgsz=self.imgsz, verbose=False)[0]
        return [
            (r.names[int(k)], c, *box)
            for box, c, k in zip(r.boxes.xyxy.tolist(), r.boxes.conf.tolist(), r.boxes.cls.tolist())
        ]


class FakeCamera:
    """Blank frames and a 'person' drifting left to right, for testing with --sim."""

    def __init__(self):
        self.t0 = time.time()

    def read(self):
        time.sleep(1 / 20)
        img = np.full((720, 1280, 3), 40, np.uint8)
        cv2.putText(img, "SIM CAMERA", (40, 80), cv2.FONT_HERSHEY_SIMPLEX, 2, (90, 90, 90), 3)
        return True, img

    def detect(self, img):
        x = 640 + 450 * math.sin((time.time() - self.t0) * 0.3)
        return [("person", 0.9, x - 90, 150, x + 90, 650), ("chair", 0.7, 100, 400, 260, 620)]


class Speaker:
    """Speaks one short phrase at a time, nearest object first, without repeating itself."""

    def __init__(self, args):
        self.args = args
        self.cmd = next((c for c in ("espeak-ng", "espeak") if shutil.which(c)), None)
        if not self.cmd:
            print("espeak-ng not found; announcements are printed (and spoken by the viewer if enabled)")
        self.busy = False
        self.seen = {}  # label -> consecutive frames it has been detected
        self.last = {}  # label -> (time spoken, distance spoken)
        self.said = {"seq": 0, "text": ""}  # latest announcement, for the viewer's laptop voice

    def consider(self, dets, frame_area):
        labels = {d["label"] for d in dets}
        self.seen = {l: self.seen.get(l, 0) + 1 for l in labels}
        if self.busy:
            return
        now = time.time()
        for d in sorted(dets, key=lambda d: 99 if d["dist"] is None else d["dist"]):
            if self.seen[d["label"]] < 3:  # ignore one-frame flickers
                continue
            if d["dist"] is None:
                # no lidar return there (too low, too high or too far): only mention big, close-looking things
                if d["area"] < 0.15 * frame_area:
                    continue
            elif d["dist"] > self.args.speak_range:
                continue
            prev = self.last.get(d["label"])
            if prev and now - prev[0] < self.args.cooldown:
                # repeat early only if it got a lot closer
                if d["dist"] is None or prev[1] is None or d["dist"] > prev[1] - 1.0:
                    continue
            self.last[d["label"]] = (now, d["dist"])
            self.say(phrase(d))
            return

    def say(self, text):
        self.said = {"seq": self.said["seq"] + 1, "text": text}
        print("say:", text)
        if not self.cmd:
            return
        self.busy = True

        def run():
            try:
                subprocess.run([self.cmd, "-s", "165", text], stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL, timeout=10)
            finally:
                self.busy = False

        threading.Thread(target=run, daemon=True).start()


def phrase(d):
    name = SPOKEN_NAME.get(d["label"], d["label"])
    a = d["angle"]
    where = "ahead" if abs(a) < 12 else ("on the left" if a > 0 else "on the right")
    if d["dist"] is None:
        return "%s %s" % (name, where)
    m = round(d["dist"] * 2) / 2  # nearest half metre
    if m < 1:
        dist = "very close"
    elif m == 1:
        dist = "1 meter"
    else:
        dist = "%g meters" % m
    return "%s %s, %s" % (name, where, dist)


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.jpeg = None
        self.dets = []
        self.fps = 0.0


def vision_loop(cam, detect, lidar, speaker, state, args):
    count, t_last = 0, time.time()
    while True:
        ok, img = cam.read()
        if not ok:
            print("camera read failed; retrying")
            time.sleep(0.5)
            continue
        h, w = img.shape[:2]
        dets = []
        for label, conf, x1, y1, x2, y2 in detect(img):
            if label not in ANNOUNCE:
                continue
            # measure across the middle half of the box so the lidar hits the object, not what's beside it
            a_left = pixel_to_angle(x1 + (x2 - x1) * 0.25, w, args.hfov, args.cam_offset)
            a_right = pixel_to_angle(x2 - (x2 - x1) * 0.25, w, args.hfov, args.cam_offset)
            dist = lidar.distance(a_left, a_right)
            dets.append({
                "label": label,
                "conf": round(conf, 2),
                "angle": round(math.degrees((a_left + a_right) / 2), 1),
                "dist": None if dist is None else round(dist, 2),
                "area": (x2 - x1) * (y2 - y1),
                "box": [x1, y1, x2, y2],
            })
        speaker.consider(dets, w * h)

        for d in dets:
            x1, y1, x2, y2 = map(int, d["box"])
            text = d["label"] + ("" if d["dist"] is None else " %.1fm" % d["dist"])
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 200, 255), 3)
            cv2.putText(img, text, (x1 + 4, max(30, y1 - 10)), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 200, 255), 3)
        small = cv2.resize(img, (640, 640 * h // w))
        jpeg = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 70])[1].tobytes()

        with state.lock:
            state.jpeg = jpeg
            state.dets = [{k: d[k] for k in ("label", "conf", "angle", "dist")} for d in dets]
        count += 1
        if time.time() - t_last >= 1.0:
            state.fps = count / (time.time() - t_last)
            count, t_last = 0, time.time()


def make_handler(state, speaker):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            if self.path.startswith("/frame.jpg"):
                # latest labelled frame; the viewer reloads it in a loop (Chrome blocks a
                # cross-port MJPEG stream inside the page, but plain JPEGs are fine)
                with state.lock:
                    jpeg = state.jpeg
                if not jpeg:
                    self.send_error(503)
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(jpeg)))
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                self.wfile.write(jpeg)
            elif self.path == "/camera":
                # MJPEG stream, to open directly in a browser tab: http://<orin-ip>:8001/camera
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                try:
                    while True:
                        with state.lock:
                            jpeg = state.jpeg
                        if jpeg:
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n"
                                             % len(jpeg) + jpeg + b"\r\n")
                            self.wfile.flush()
                        time.sleep(1 / 12)
                except (BrokenPipeError, ConnectionResetError):
                    pass
            elif self.path == "/dets":
                # Server-Sent Events, read by viewer.html (served from port 8000, hence the CORS header)
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                try:
                    while True:
                        with state.lock:
                            frame = {"dets": state.dets, "fps": round(state.fps, 1), "said": speaker.said}
                        self.wfile.write(b"data: " + json.dumps(frame).encode() + b"\n\n")
                        self.wfile.flush()
                        time.sleep(1 / 10)
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self.send_error(404)

    return Handler


def open_camera(index):
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2) if os.name == "posix" else cv2.VideoCapture(index)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))  # needed for 30 fps at 720p over USB 2
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # always process the newest frame
    if not cap.isOpened():
        raise SystemExit("could not open camera %d (try --camera 1, or check `ls /dev/video*`)" % index)
    return cap


def main():
    engine = os.path.join(HERE, "yolo11n.engine")
    p = argparse.ArgumentParser()
    p.add_argument("--sim", action="store_true", help="fake camera and detections, no YOLO needed")
    p.add_argument("--camera", type=int, default=0, help="webcam index (/dev/videoN)")
    p.add_argument("--model", default=engine if os.path.exists(engine) else "yolo11n.pt",
                   help="YOLO model; uses yolo11n.engine (TensorRT) when it has been exported")
    p.add_argument("--conf", type=float, default=0.45, help="minimum detection confidence")
    p.add_argument("--imgsz", type=int, default=640)
    p.add_argument("--hfov", type=float, default=50,
                   help="camera horizontal field of view in degrees (see README to measure it)")
    p.add_argument("--cam-offset", type=float, default=0,
                   help="degrees to add if labels sit left (+) or right (-) of the lidar points")
    p.add_argument("--speak-range", type=float, default=4.0, help="only announce objects closer than this (m)")
    p.add_argument("--cooldown", type=float, default=6.0, help="seconds before repeating the same object")
    p.add_argument("--lidar-url", default="http://127.0.0.1:8000/stream")
    p.add_argument("--port", type=int, default=8001)
    args = p.parse_args()

    if args.sim:
        cam = FakeCamera()
        detect = cam.detect
    else:
        print("loading", args.model)
        detect = Detector(args.model, args.conf, args.imgsz)
        cam = open_camera(args.camera)

    state = State()
    lidar = LidarFeed(args.lidar_url)
    speaker = Speaker(args)

    def run_vision():
        while True:
            try:
                vision_loop(cam, detect, lidar, speaker, state, args)
            except Exception as e:
                print("vision error:", e)
                time.sleep(1)

    threading.Thread(target=run_vision, daemon=True).start()
    server = ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(state, speaker))
    server.daemon_threads = True
    print("vision running; open the usual viewer at http://<this-device-ip>:8000")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
