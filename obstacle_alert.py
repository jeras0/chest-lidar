"""Chest lidar obstacle alert for the Jetson Orin Nano.

  - reads scans from the Hokuyo UST-10LX
  - beeps a buzzer on the 40-pin header, faster as obstacles get closer
  - serves a live view at http://<orin-ip>:8000 for any laptop/phone on the same Wi-Fi

Run on the Orin:    sudo python3 obstacle_alert.py
Test anywhere:      python3 obstacle_alert.py --sim      (fake lidar, no buzzer needed)
"""
import argparse
import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

from lidar_scip import FakeLidar, Lidar, step_to_angle

# Alert zones, closest first: (max distance in metres, beep on seconds, beep off seconds)
# off == 0 means a continuous tone. Distances can be changed with --zones.
ZONES = [
    (0.75, 1.0, 0.0),  # danger: solid tone
    (1.5, 0.08, 0.12), # close: fast beeps
    (2.0, 0.08, 0.45), # caution: slow beeps
]

HERE = os.path.dirname(os.path.abspath(__file__))


class Buzzer:
    """Active buzzer (or vibration motor) driven through a transistor on a header pin."""

    def __init__(self, pin):
        self.pin = pin
        try:
            import Jetson.GPIO as GPIO
            GPIO.setmode(GPIO.BOARD)
            GPIO.setwarnings(False)
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
            self.gpio = GPIO
        except Exception as e:  # laptop, or GPIO permissions not set up
            print("buzzer disabled (%s); printing alerts instead" % e)
            self.gpio = None

    def set(self, on):
        if self.gpio:
            self.gpio.output(self.pin, self.gpio.HIGH if on else self.gpio.LOW)

    def cleanup(self):
        if self.gpio:
            self.set(False)
            self.gpio.cleanup()


class State:
    """Latest scan and alert info, shared between threads."""

    def __init__(self):
        self.lock = threading.Lock()
        self.ranges = []
        self.nearest = None  # metres, in the front sector
        self.zone = None     # index into ZONES, or None when clear
        self.hz = 0.0


def nearest_in_front(ranges, fov_deg):
    """Robust nearest distance (m) in a wedge fov_deg wide, centred straight ahead.
    Uses the 5th closest valid reading so a single noisy point does not trigger an alert."""
    half = math.radians(fov_deg / 2)
    vals = sorted(
        r for i, r in enumerate(ranges)
        if r >= 20 and abs(step_to_angle(i)) <= half
    )
    if not vals:
        return None
    return vals[min(4, len(vals) - 1)] / 1000.0


def zone_for(distance):
    if distance is None:
        return None
    for i, (limit, _, _) in enumerate(ZONES):
        if distance <= limit:
            return i
    return None


def lidar_loop(lidar, state, args):
    count, t_last = 0, time.time()
    while True:
        ranges = lidar.scan()
        near = nearest_in_front(ranges, args.fov)
        with state.lock:
            state.ranges = ranges
            state.nearest = near
            state.zone = zone_for(near)
        count += 1
        if time.time() - t_last >= 1.0:
            state.hz = count / (time.time() - t_last)
            count, t_last = 0, time.time()


def buzzer_loop(buzzer, state):
    last_zone = "start"
    while True:
        with state.lock:
            zone, near = state.zone, state.nearest
        if zone != last_zone:
            print("zone:", "clear" if zone is None else ZONES[zone][0], "m  nearest:",
                  "none" if near is None else "%.2f m" % near)
            last_zone = zone
        if zone is None:
            buzzer.set(False)
            time.sleep(0.05)
            continue
        _, on, off = ZONES[zone]
        buzzer.set(True)
        time.sleep(on if off else 0.05)
        if off:
            buzzer.set(False)
            time.sleep(off)


def make_handler(state, args):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            if self.path == "/":
                with open(os.path.join(HERE, "viewer.html"), "rb") as f:
                    body = f.read()
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == "/stream":
                # Server-Sent Events: one JSON frame per message, ~15 fps
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                try:
                    while True:
                        with state.lock:
                            frame = {
                                "r": state.ranges[::2],  # every 0.5 deg is plenty for display
                                "step": 2,
                                "nearest": state.nearest,
                                "zone": state.zone,
                                "zones": [z[0] for z in ZONES],
                                "fov": args.fov / 2,  # viewer wants the half-angle
                                "hz": round(state.hz, 1),
                            }
                        self.wfile.write(b"data: " + json.dumps(frame).encode() + b"\n\n")
                        self.wfile.flush()
                        time.sleep(1 / 15)
                except (BrokenPipeError, ConnectionResetError):
                    pass
            elif self.path.startswith("/set?"):
                # live tuning from the viewer, e.g. /set?fov=120
                q = parse_qs(urlparse(self.path).query)
                try:
                    if "fov" in q:
                        args.fov = max(1.0, min(270.0, float(q["fov"][0])))
                        print("alert wedge now %.0f deg wide" % args.fov)
                except ValueError:
                    self.send_error(400)
                    return
                self.send_response(204)
                self.end_headers()
            else:
                self.send_error(404)

    return Handler


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--sim", action="store_true", help="use a simulated lidar")
    p.add_argument("--lidar-ip", default="192.168.0.10")
    p.add_argument("--pin", type=int, default=7, help="BOARD pin number for the buzzer")
    p.add_argument("--fov", type=float, default=90,
                   help="total width in degrees of the forward wedge that triggers alerts (max 270)")
    p.add_argument("--zones", default=None,
                   help="alert distances in metres, closest first: stop,close,caution (e.g. 0.75,1.5,2.0)")
    p.add_argument("--port", type=int, default=8000)
    args = p.parse_args()
    if args.zones:
        dists = [float(x) for x in args.zones.split(",")]
        if len(dists) != len(ZONES) or dists != sorted(dists):
            p.error("--zones needs %d increasing distances, e.g. 0.75,1.5,2.0" % len(ZONES))
        ZONES[:] = [(d, on, off) for d, (_, on, off) in zip(dists, ZONES)]
    print("alert wedge %.0f deg wide, zones (m): %s" % (args.fov, [z[0] for z in ZONES]))

    state = State()
    buzzer = Buzzer(args.pin)

    while True:  # keep retrying so the device recovers if the cable is bumped
        try:
            lidar = FakeLidar() if args.sim else Lidar(args.lidar_ip)
            break
        except OSError as e:
            print("waiting for lidar at %s: %s" % (args.lidar_ip, e))
            time.sleep(2)

    def run_lidar():
        nonlocal lidar
        while True:
            try:
                lidar_loop(lidar, state, args)
            except Exception as e:
                print("lidar error:", e, "- reconnecting")
                with state.lock:
                    state.zone = None
                lidar.close()
                time.sleep(1)
                try:
                    lidar = Lidar(args.lidar_ip)
                except OSError:
                    pass

    threading.Thread(target=run_lidar, daemon=True).start()
    threading.Thread(target=buzzer_loop, args=(buzzer, state), daemon=True).start()

    server = ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(state, args))
    server.daemon_threads = True
    print("live view: http://<this-device-ip>:%d" % args.port)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        buzzer.cleanup()
        lidar.close()


if __name__ == "__main__":
    main()
