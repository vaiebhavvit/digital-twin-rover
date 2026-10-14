"""Rover client — runs on the Jetson Orin Nano (or a Raspberry Pi 5).

The complete on-robot loop:

    pull the route from the twin
    for each waypoint:
        drive towards it, reading the ultrasonic sensor as you go
        if the twin says STOP  -> halt, wait recheck_interval, read again
        if the twin says GO    -> carry on
        on arrival at an inspect waypoint: turn, run YOLO, report the scan

The route is NOT hard-coded here. It comes from `GET /api/route`, which
serves whatever is in `config/route.yaml`. Edit that one file and the
dashboard, the simulator and this client all follow the new path.

    pip install ultralytics opencv-python pyserial requests
    python3 tools/rover_client.py --server http://192.168.1.50:8000 \
                                  --port /dev/ttyUSB0 --model best.pt

Only two methods talk to hardware — `read_serial` and `drive_segment`. Adapt
those to your firmware; `firmware/rover_firmware.ino` implements the protocol
they expect.

Safety note: the rover stops on its own the moment the sensor reads short,
before it asks the twin. Never make a stop depend on a network round trip.
"""

from __future__ import annotations

import argparse
import math
import time
import sys

import requests
import serial

ARRIVAL_RADIUS = 0.12        # metres — close enough to call a waypoint reached
HEADING_TOLERANCE = 0.06     # radians
CONTROL_HZ = 20
POSE_REPORT_HZ = 8
READING_HZ = 4               # how often to send the ultrasonic to the twin


class RoverClient:
    def __init__(self, args):
        self.server = args.server.rstrip("/")
        self.session = requests.Session()
        if args.token:
            self.session.headers["X-Twin-Token"] = args.token

        self.serial = serial.Serial(args.port, args.baud, timeout=0.1)
        time.sleep(2)                      # the board resets when the port opens

        self.model = None
        self.cap = None
        if not args.no_vision:
            import cv2
            from ultralytics import YOLO
            self.model = YOLO(args.model)
            self.cap = cv2.VideoCapture(args.camera)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            print(f"  vision : {list(self.model.names.values())}")

        self.route = self.fetch_route()
        self.stop_distance = self.route.get("obstacle_stop_distance", 0.40)
        self.recheck_interval = self.route.get("recheck_interval", 3.0)
        self.clear_margin = self.route.get("clear_margin", 0.10)

        start = self.route["waypoints"][0]
        self.x, self.y = start["x"], start["y"]
        self.theta = math.radians(start.get("heading") or 0)
        self.battery = 100.0
        self.lap = 0
        self.range_m = 4.0                 # last ultrasonic reading
        self.last_pose_sent = 0.0
        self.last_reading_sent = 0.0
        self.save_frames = args.save_frames

    # ------------------------------------------------------------------
    # The twin
    # ------------------------------------------------------------------
    def fetch_route(self) -> dict:
        r = self.session.get(f"{self.server}/api/route", timeout=10)
        r.raise_for_status()
        route = r.json()
        print(f"  route  : {route['name']} — {len(route['waypoints'])} waypoints, "
              f"{route['inspect_stops']} inspection stops, "
              f"{route['total_length_m']} m")
        print(f"  sensor : stop under {route.get('obstacle_stop_distance')} m, "
              f"recheck every {route.get('recheck_interval')} s")
        for w in route.get("warnings", []):
            print(f"  WARNING: {w}")
        return route

    def post(self, path, payload, want_reply=False):
        try:
            r = self.session.post(self.server + path, json=payload, timeout=3)
            return r.json() if want_reply and r.status_code < 400 else None
        except requests.RequestException as e:
            # Keep driving. The twin catches up when the network returns.
            print(f"  ! twin unreachable: {type(e).__name__}")
            return None

    def report_pose(self, state, waypoint=None, target_shelf=None, force=False):
        now = time.time()
        if not force and now - self.last_pose_sent < 1.0 / POSE_REPORT_HZ:
            return
        self.last_pose_sent = now
        self.post("/api/rover/pose", {
            "pose": {"x": round(self.x, 3), "y": round(self.y, 3),
                     "theta": round(self.theta, 4), "stamp": now},
            "telemetry": {"battery": round(self.battery, 1), "state": state,
                          "lap": self.lap, "waypoint": waypoint,
                          "target_shelf": target_shelf},
        })

    def report_reading(self, stopped=False, force=False) -> dict | None:
        """Send the current range to the twin; it answers go / stop."""
        now = time.time()
        if not force and now - self.last_reading_sent < 1.0 / READING_HZ:
            return None
        self.last_reading_sent = now
        return self.post("/api/rover/ultrasonic", {
            "distance": round(self.range_m, 3), "sensor": "front",
            "stopped": stopped,
            "pose": {"x": round(self.x, 3), "y": round(self.y, 3),
                     "theta": round(self.theta, 4)},
            "stamp": now,
        }, want_reply=True)

    # ------------------------------------------------------------------
    # Hardware — adapt these two to your firmware
    # ------------------------------------------------------------------
    def send(self, cmd: str):
        self.serial.write((cmd + "\n").encode())
        self.serial.flush()

    def read_serial(self):
        """Drain the serial buffer and fold every line into the rover state.

        Lines the firmware sends:
            P <dx> <dy> <dtheta>   odometry delta, metres and degrees
            U <cm>                 front ultrasonic range
            B <volts>              battery pack voltage
        """
        while self.serial.in_waiting:
            line = self.serial.readline().decode(errors="ignore").strip()
            if not line:
                continue
            parts = line.split()
            try:
                if parts[0] == "P" and len(parts) >= 4:
                    dx, dy, dth = float(parts[1]), float(parts[2]), float(parts[3])
                    # Odometry deltas arrive in the robot's own frame.
                    self.x += dx * math.cos(self.theta) - dy * math.sin(self.theta)
                    self.y += dx * math.sin(self.theta) + dy * math.cos(self.theta)
                    self.theta = (self.theta + math.radians(dth)) % (2 * math.pi)
                elif parts[0] == "U" and len(parts) >= 2:
                    cm = float(parts[1])
                    # The HC-SR04 reports 0 or a huge number when it hears no
                    # echo at all. Treat that as "nothing in range", never as
                    # "something touching the bumper".
                    self.range_m = 4.0 if cm <= 0 or cm > 400 else cm / 100.0
                elif parts[0] == "B" and len(parts) >= 2:
                    volts = float(parts[1])
                    self.battery = max(0.0, min(100.0,
                                                (volts - 9.6) / (12.6 - 9.6) * 100))
            except (ValueError, IndexError):
                continue

    # ------------------------------------------------------------------
    # The obstacle cycle
    # ------------------------------------------------------------------
    def path_is_clear(self, waypoint_id) -> bool:
        """Local stop first, then ask the twin. Blocks while the path is red."""
        # 1. Stop on our own reading. This must not wait for the network.
        if self.range_m > self.stop_distance:
            self.report_reading()
            return True

        self.send("S")
        decision = self.report_reading(stopped=True, force=True)
        wait = (decision or {}).get("recheck_in_s") or self.recheck_interval
        attempts = (decision or {}).get("attempts", 1)
        print(f"  [STOP] obstacle {self.range_m:.2f} m ahead before {waypoint_id} "
              f"— holding, recheck in {wait:.0f} s")

        # 2. Hold, re-read, repeat until the way is properly clear.
        while True:
            waited = 0.0
            while waited < wait:
                self.read_serial()
                self.report_pose("blocked", waypoint_id)
                time.sleep(1.0 / CONTROL_HZ)
                waited += 1.0 / CONTROL_HZ

            self.read_serial()
            # Hysteresis, same rule the twin applies: clearing needs more room
            # than stopping did, so a reading sitting on the threshold cannot
            # make the rover lurch stop-go-stop-go.
            locally_clear = self.range_m > self.stop_distance + self.clear_margin
            decision = self.report_reading(stopped=True, force=True)

            if decision is not None:
                if decision["action"] == "go":
                    print(f"  [GO]   {decision['message']}")
                    return True
                attempts = decision.get("attempts", attempts + 1)
                wait = decision.get("recheck_in_s") or self.recheck_interval
                print(f"  [STOP] check {attempts} — still blocked "
                      f"at {self.range_m:.2f} m")
            else:
                # No twin to ask: fall back to the local rule.
                if locally_clear:
                    print(f"  [GO]   clear at {self.range_m:.2f} m (offline)")
                    return True
                wait = self.recheck_interval

    # ------------------------------------------------------------------
    # Motion
    # ------------------------------------------------------------------
    def turn_to(self, heading_rad, waypoint_id):
        if heading_rad is None:
            return
        while True:
            self.read_serial()
            diff = (heading_rad - self.theta + math.pi) % (2 * math.pi) - math.pi
            if abs(diff) < HEADING_TOLERANCE:
                break
            self.send(f"T{math.degrees(diff):.0f}")
            self.report_pose("moving", waypoint_id)
            time.sleep(1.0 / CONTROL_HZ)
        self.send("S")

    def drive_segment(self, wp, speed_mps):
        """Drive to one waypoint, pausing for anything in the way."""
        wid = wp["id"]
        target_shelf = wp.get("shelf") if wp.get("action") == "inspect" else None

        while True:
            self.read_serial()

            if not self.path_is_clear(wid):
                continue

            dx, dy = wp["x"] - self.x, wp["y"] - self.y
            dist = math.hypot(dx, dy)
            if dist < ARRIVAL_RADIUS:
                self.send("S")
                break

            bearing = math.atan2(dy, dx)
            diff = (bearing - self.theta + math.pi) % (2 * math.pi) - math.pi
            if abs(diff) > 0.25:
                self.send(f"T{math.degrees(diff):.0f}")
            else:
                # Ease off as the waypoint approaches so the stop is clean.
                v = speed_mps * (0.4 if dist < 0.4 else 1.0)
                self.send(f"F{int(v * 1000)}")

            self.report_pose("moving", wid, target_shelf)
            time.sleep(1.0 / CONTROL_HZ)

    # ------------------------------------------------------------------
    # Vision
    # ------------------------------------------------------------------
    def detect(self):
        if self.model is None or self.cap is None:
            return [], None
        import cv2
        ok, frame = self.cap.read()
        if not ok:
            print("  camera read failed")
            return [], None

        result = self.model(frame, verbose=False)[0]
        counts, confs = {}, {}
        for box in result.boxes:
            conf = float(box.conf[0])
            if conf < 0.55:
                continue
            name = self.model.names[int(box.cls[0])]
            counts[name] = counts.get(name, 0) + 1
            confs.setdefault(name, []).append(conf)
        detections = [{"product": n, "qty": q,
                       "confidence": round(sum(confs[n]) / len(confs[n]), 2)}
                      for n, q in counts.items()]
        return detections, (result, cv2)

    def inspect(self, wp):
        shelf_id = wp.get("shelf")
        self.send("S")
        self.report_pose("inspecting", wp["id"], shelf_id, force=True)
        time.sleep(0.8)                       # let the camera settle

        detections, extra = self.detect()
        if extra and self.save_frames:
            result, cv2 = extra
            import os
            os.makedirs("scans", exist_ok=True)
            cv2.imwrite(f"scans/{shelf_id}_{int(time.time())}.jpg", result.plot())

        self.post("/api/inspection", {
            "shelf_id": shelf_id,
            "detections": detections,
            "pose": {"x": round(self.x, 3), "y": round(self.y, 3),
                     "theta": round(self.theta, 4)},
            "telemetry": {"battery": round(self.battery, 1),
                          "state": "inspecting", "lap": self.lap},
        })
        summary = ", ".join(f"{d['product']} x{d['qty']}" for d in detections) or "nothing"
        print(f"  {shelf_id}: {summary}")

        pause = wp.get("pause")
        time.sleep(pause if pause is not None else 2.5)

    # ------------------------------------------------------------------
    def run(self):
        wps = self.route["waypoints"]
        default_speed = self.route.get("default_speed", 0.45)
        try:
            while True:
                self.lap += 1
                print(f"\n--- lap {self.lap} | battery {self.battery:.0f}% ---")
                for i, wp in enumerate(wps):
                    if i == 0 and self.lap == 1:
                        continue
                    self.drive_segment(wp, wp.get("speed") or default_speed)
                    heading = wp.get("heading")
                    self.turn_to(math.radians(heading) if heading is not None else None,
                                 wp["id"])

                    action = wp.get("action", "move")
                    if action == "inspect":
                        self.inspect(wp)
                    elif action in ("dock", "wait"):
                        self.send("S")
                        self.report_pose("docked" if action == "dock" else "stopped",
                                         wp["id"], force=True)
                        time.sleep(wp.get("pause") or 0)

                if not self.route.get("loop", True):
                    print("\nroute finished")
                    self.report_pose("stopped", force=True)
                    return
        except KeyboardInterrupt:
            pass
        finally:
            self.send("S")
            if self.cap:
                self.cap.release()
            self.serial.close()
            self.report_pose("stopped", force=True)
            print("\nrover stopped")


def _utf8_console():
    """Make stdout safe for the arrows and dashes this project prints.

    On Windows a redirected stream (`python run.py > server.log`) uses the
    locale encoding, cp1252, which has no U+2192. Without this, logging to a
    file crashes on the first arrow. Harmless everywhere else.
    """
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, ValueError):
            pass           # already UTF-8, or not a reconfigurable stream


def main():
    _utf8_console()
    ap = argparse.ArgumentParser()
    ap.add_argument("--server", default="http://localhost:8000")
    ap.add_argument("--token", default="")
    ap.add_argument("--port", default="/dev/ttyUSB0", help="Arduino / ESP32 serial port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--model", default="best.pt", help="your trained YOLO weights")
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--no-vision", action="store_true",
                    help="drive the route without running YOLO (navigation testing)")
    ap.add_argument("--save-frames", action="store_true",
                    help="write annotated frames to scans/ as evidence")
    args = ap.parse_args()

    print("Rover client")
    RoverClient(args).run()


if __name__ == "__main__":
    main()
