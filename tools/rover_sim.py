"""Rover simulator — drives the oval with no hardware.

Reads the same `config/route.yaml` the real rover follows, drives the oval
waypoint by waypoint, stops to inspect bays, and runs the full obstacle
cycle against the twin:

    obstacle within stop distance  ->  POST the reading, twin says STOP,
                                       strip ahead turns RED, rover halts
    wait recheck_interval seconds  ->  POST another reading
    still blocked                  ->  stays RED, waits again
    clear                          ->  twin says GO, strip turns GREEN,
                                       rover moves off

It speaks exactly the HTTP contract the real rover uses, so the dashboard
cannot tell them apart. Use it to develop, to demo, and as the fallback if
the hardware misbehaves on review day.

    python -m tools.rover_sim
    python -m tools.rover_sim --speed 3          # 3x real time for a demo
    python -m tools.rover_sim --obstacle-rate 0  # no obstacles
    python -m tools.rover_sim --drop A3          # park an obstacle at bay A3
"""

from __future__ import annotations

import argparse
import math
import random
import sys
import time
from pathlib import Path

import os

import requests
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from twin.config import settings                      # noqa: E402
from twin.models import LayoutDef, RouteDef           # noqa: E402
from twin.route import Route, generate_oval           # noqa: E402

TICK = 0.12                 # seconds of simulated time per control step
ODOM_NOISE = 0.004          # metres of encoder noise per metre travelled
SENSOR_NOISE = 0.015        # metres of ultrasonic noise
SENSOR_MAX = 4.0            # HC-SR04 practical ceiling, metres
READING_EVERY = 3           # ticks between ultrasonic reads while driving

DEBUG = bool(os.getenv("SIM_DEBUG"))


class Obstacle:
    """Something sitting in the aisle, in the way."""

    def __init__(self, x, y, clears_at, label="box"):
        self.x, self.y = x, y
        self.clears_at = clears_at          # None = never clears on its own
        self.label = label

    def alive(self, now):
        return self.clears_at is None or now < self.clears_at


class SimRover:
    def __init__(self, route, layout, server, speed_mult, obstacle_rate, drops,
                 drop_clears=12.0):
        self.route = route
        self.layout = layout
        self.server = server.rstrip("/")
        self.speed_mult = speed_mult
        self.obstacle_rate = obstacle_rate

        self.session = requests.Session()
        if settings.ingest_token:
            self.session.headers["X-Twin-Token"] = settings.ingest_token

        start = route.waypoints[0]
        self.x, self.y = start.x, start.y
        self.theta = math.radians(start.heading or 0)
        self.battery = 100.0
        self.lap = 0
        self.rng = random.Random()
        self.tick_count = 0

        self.stock = {s.shelf_id: {p: float(q) for p, q in s.products.items()}
                      for s in layout.shelves}
        self.shelves = {s.shelf_id: s for s in layout.shelves}
        self.strays: dict[str, str] = {}

        self.obstacles: list[Obstacle] = []
        self.pending_drops: list[str] = []
        self.drop_clears = drop_clears
        for shelf_id in drops:
            if route.shelf_waypoint(shelf_id):
                self.pending_drops.append(shelf_id)
                print(f"  will drop an obstacle before {shelf_id}"
                      + (f", clearing after {drop_clears:.0f} s"
                         if drop_clears > 0 else " (never clears)"))
            else:
                print(f"  ! --drop {shelf_id}: no inspection stop for that bay")

    # ---------------- transport ----------------
    def post(self, path, payload, expect_json=False):
        try:
            r = self.session.post(self.server + path, json=payload, timeout=4)
            return r.json() if expect_json and r.status_code < 400 else None
        except requests.RequestException as e:
            print(f"  ! twin unreachable: {type(e).__name__}")
            return None

    def publish_pose(self, state, waypoint=None, target_shelf=None):
        self.post("/api/rover/pose", {
            "pose": {"x": round(self.x, 3), "y": round(self.y, 3),
                     "theta": round(self.theta, 4), "stamp": time.time()},
            "telemetry": {"battery": round(self.battery, 1), "state": state,
                          "lap": self.lap, "speed": round(self.current_speed, 2),
                          "waypoint": waypoint, "target_shelf": target_shelf},
        })

    # ---------------- the simulated HC-SR04 ----------------
    current_speed = 0.0

    def read_ultrasonic(self) -> float:
        """Distance to the nearest obstacle in a narrow cone ahead."""
        now = time.time()
        self.obstacles = [o for o in self.obstacles if o.alive(now)]

        best = SENSOR_MAX
        for o in self.obstacles:
            dx, dy = o.x - self.x, o.y - self.y
            dist = math.hypot(dx, dy)
            if dist > SENSOR_MAX:
                continue
            # Only things roughly in front are seen: a real HC-SR04 has about
            # a 15 degree half-angle.
            bearing = math.atan2(dy, dx)
            off = abs((bearing - self.theta + math.pi) % (2 * math.pi) - math.pi)
            if off < math.radians(18):
                best = min(best, dist)
        return max(0.02, best + self.rng.gauss(0, SENSOR_NOISE))

    def place_drop(self, shelf_id):
        """Put a requested obstacle in the aisle just before a named bay.

        Clamped to sit at least a short way AHEAD of where the rover is now:
        a bay near the start of the lap would otherwise put the crate at a
        negative distance, which wraps round to the far end of the oval and
        is never seen.
        """
        w = self.route.shelf_waypoint(shelf_id)
        bay_d = self.route.locate(w.x, w.y)["travelled_m"]
        here_d = self.route.locate(self.x, self.y)["travelled_m"]
        if bay_d < here_d:                       # the bay is over the lap seam
            bay_d += self.route.total_length
        drop_d = max(here_d + 0.9, bay_d - 1.1)
        if drop_d >= bay_d:                      # no room before the bay
            drop_d = here_d + 0.9
        ox, oy = self.route.point_at(drop_d)
        clears = None if self.drop_clears <= 0 else time.time() + self.drop_clears
        self.obstacles.append(Obstacle(ox, oy, clears, f"crate at {shelf_id}"))
        print(f"  [world] a crate is sitting in the aisle "
              f"{drop_d - here_d:.1f} m ahead, before {shelf_id}")

    def maybe_spawn_obstacle(self):
        """Something wanders into the aisle a little way ahead of the rover."""
        if self.obstacle_rate <= 0 or self.rng.random() > self.obstacle_rate:
            return
        d = self.route.locate(self.x, self.y)["travelled_m"]
        ahead = self.rng.uniform(1.2, 2.6)
        ox, oy = self.route.point_at(d + ahead)
        clears_in = self.rng.uniform(4.0, 11.0) / self.speed_mult
        self.obstacles.append(Obstacle(ox, oy, time.time() + clears_in, "trolley"))
        print(f"  [world] a trolley rolls into the aisle {ahead:.1f} m ahead")

    # ---------------- the obstacle cycle ----------------
    def check_path(self, waypoint_id) -> bool:
        """Send a reading, obey the twin. Returns True if clear to proceed."""
        reading = self.read_ultrasonic()
        if DEBUG:
            print(f"    [dbg] obs={len(self.obstacles)} "
                  f"pos=({self.x:.2f},{self.y:.2f}) "
                  f"th={math.degrees(self.theta):+.0f} read={reading:.2f}",
                  flush=True)
        decision = self.post("/api/rover/ultrasonic", {
            "distance": round(reading, 3), "sensor": "front",
            "pose": {"x": round(self.x, 3), "y": round(self.y, 3),
                     "theta": round(self.theta, 4)},
            "stamp": time.time(),
        }, expect_json=True)

        if decision is None:                     # twin unreachable
            # Fail safe: stop on a close reading even with no twin to ask.
            return reading > self.route.stop_distance

        if decision["action"] == "go":
            return True

        # --- blocked: hold position and re-read on the twin's schedule ---
        self.current_speed = 0.0
        wait = decision.get("recheck_in_s") or self.route.recheck_interval
        print(f"  [STOP] {decision['message']}")

        while True:
            # The recheck interval is wall-clock and is never scaled by
            # --speed: it is how long the real rover waits before trusting
            # the sensor again, so speeding up the demo must not shorten it.
            waited = 0.0
            while waited < wait:
                self.publish_pose("blocked", waypoint_id)
                time.sleep(TICK)
                waited += TICK

            reading = self.read_ultrasonic()
            decision = self.post("/api/rover/ultrasonic", {
                "distance": round(reading, 3), "sensor": "front",
                "stopped": True,
                "pose": {"x": round(self.x, 3), "y": round(self.y, 3),
                         "theta": round(self.theta, 4)},
            }, expect_json=True)

            if decision is None:
                if reading > self.route.stop_distance:
                    return True
                wait = self.route.recheck_interval
                continue

            if decision["action"] == "go":
                print(f"  [GO]   {decision['message']}")
                return True
            print(f"  [STOP] check {decision['attempts']} — "
                  f"still blocked at {reading:.2f} m")
            wait = decision.get("recheck_in_s") or self.route.recheck_interval

    # ---------------- motion ----------------
    def turn_to(self, heading_rad, waypoint=None):
        if heading_rad is None:
            return
        self.current_speed = 0.0
        while True:
            diff = (heading_rad - self.theta + math.pi) % (2 * math.pi) - math.pi
            if abs(diff) < 0.05:
                break
            self.theta += max(-2.0 * TICK, min(2.0 * TICK, diff))
            self.publish_pose("moving", waypoint)
            time.sleep(TICK / self.speed_mult)

    def drive_to(self, wp, speed):
        wid = wp.id
        target_shelf = wp.shelf if wp.action == "inspect" else None
        if wp.shelf and wp.shelf in self.pending_drops:
            self.pending_drops.remove(wp.shelf)
            self.place_drop(wp.shelf)
        else:
            self.maybe_spawn_obstacle()

        while True:
            dx, dy = wp.x - self.x, wp.y - self.y
            dist = math.hypot(dx, dy)
            if dist < 0.06:
                break

            self.tick_count += 1
            if self.tick_count % READING_EVERY == 0:
                self.check_path(wid)        # blocks here while the path is red

            target = math.atan2(dy, dx)
            diff = (target - self.theta + math.pi) % (2 * math.pi) - math.pi
            self.theta += max(-2.2 * TICK, min(2.2 * TICK, diff))

            step = min(speed * TICK, dist)
            self.current_speed = step / TICK
            self.x += math.cos(self.theta) * step
            self.y += math.sin(self.theta) * step

            # Encoder drift: the twin shows this as "off route by N cm".
            self.x += self.rng.gauss(0, ODOM_NOISE * step)
            self.y += self.rng.gauss(0, ODOM_NOISE * step)

            self.battery = max(4.0, self.battery - step * 0.09)
            self.publish_pose("moving", wid, target_shelf)
            time.sleep(TICK / self.speed_mult)

        self.current_speed = 0.0

    # ---------------- inspection ----------------
    def inspect(self, wp):
        shelf_id = wp.shelf
        shelf = self.shelves.get(shelf_id)
        if shelf is None:
            print(f"  ! waypoint {wp.id} inspects unknown bay {shelf_id}")
            return

        self.publish_pose("inspecting", wp.id, shelf_id)
        time.sleep(0.4 / self.speed_mult)

        detections = []
        for product in shelf.products:
            self.stock[shelf_id][product] = max(
                0.0, self.stock[shelf_id][product] - self.rng.uniform(0.15, 1.3))
            detections.append({
                "product": product,
                "qty": int(round(self.stock[shelf_id][product])),
                "confidence": round(self.rng.uniform(0.86, 0.98), 2),
            })

        stray = self.strays.get(shelf_id)
        if stray:
            detections.append({"product": stray, "qty": 1,
                               "confidence": round(self.rng.uniform(0.78, 0.94), 2)})

        self.post("/api/inspection", {
            "shelf_id": shelf_id, "detections": detections,
            "pose": {"x": round(self.x, 3), "y": round(self.y, 3),
                     "theta": round(self.theta, 4)},
            "telemetry": {"battery": round(self.battery, 1),
                          "state": "inspecting", "lap": self.lap},
        })

        note = f"  <- misplaced {stray}" if stray else ""
        summary = ", ".join(f"{d['product']} x{d['qty']}" for d in detections)
        print(f"  scan {shelf_id}: {summary}{note}")
        time.sleep(self.route.pause_for(wp) / self.speed_mult)

    # ---------------- warehouse events ----------------
    def world_events(self):
        r = self.rng.random()
        if r < 0.32 and len(self.strays) < 2:
            src, dst = self.rng.sample(list(self.shelves), 2)
            product = next(iter(self.shelves[src].products))
            self.strays[dst] = product
            print(f"  [event] staff left {product} on {dst}")
        elif r < 0.48 and self.strays:
            shelf = self.rng.choice(list(self.strays))
            print(f"  [event] {self.strays.pop(shelf)} collected from {shelf}")
        elif r < 0.72:
            low = [(s, p) for s, ps in self.stock.items() for p, v in ps.items()
                   if v < 0.4 * self.shelves[s].products[p]]
            if low:
                s, p = self.rng.choice(low)
                self.stock[s][p] = float(self.shelves[s].products[p])
                print(f"  [event] {p} restocked on {s}")

    # ---------------- main loop ----------------
    def run(self):
        wps = self.route.waypoints
        print(f"  route  : {self.route.name}")
        print(f"  path   : {len(wps)} waypoints, {len(self.route.inspect_stops)} "
              f"inspection stops, {self.route.total_length:.1f} m per lap")
        print(f"  sensor : stop under {self.route.stop_distance:.2f} m, "
              f"recheck every {self.route.recheck_interval:.0f} s")
        try:
            while True:
                self.lap += 1
                print(f"\n--- lap {self.lap} | battery {self.battery:.0f}% ---")
                self.world_events()

                for i, wp in enumerate(wps):
                    if i == 0 and self.lap == 1:
                        continue
                    self.drive_to(wp, self.route.speed_for(i))
                    self.turn_to(wp.heading_rad, wp.id)

                    if wp.action == "inspect":
                        self.inspect(wp)
                    elif wp.action == "dock":
                        self.publish_pose("docked", wp.id)
                        time.sleep(self.route.pause_for(wp) / self.speed_mult)
                    elif wp.action == "wait":
                        self.publish_pose("stopped", wp.id)
                        time.sleep(self.route.pause_for(wp) / self.speed_mult)

                if not self.route.loop:
                    print("\nroute finished (loop: false)")
                    self.publish_pose("stopped")
                    return

                if self.battery < 15:
                    print("  [event] docking to charge")
                    while self.battery < 95:
                        self.battery = min(100.0, self.battery + 8)
                        self.publish_pose("charging")
                        time.sleep(0.4 / self.speed_mult)
        except KeyboardInterrupt:
            self.publish_pose("stopped")
            print("\nsimulator stopped")


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
    ap.add_argument("--speed", type=float, default=1.0,
                    help="time multiplier; 3 is a good pace for a live demo")
    ap.add_argument("--obstacle-rate", type=float, default=0.22,
                    help="chance per leg that something blocks the path (0 = none)")
    ap.add_argument("--drop", action="append", default=[], metavar="BAY",
                    help="drop an obstacle in the aisle before this bay, "
                         "e.g. --drop A3 (repeatable)")
    ap.add_argument("--drop-clears", type=float, default=12.0,
                    help="seconds before a dropped obstacle is removed; "
                         "0 leaves it there for good")
    ap.add_argument("--route", default=str(settings.route_file))
    ap.add_argument("--layout", default=str(settings.layout_file))
    args = ap.parse_args()

    layout = LayoutDef(**yaml.safe_load(Path(args.layout).read_text(encoding="utf-8")))
    definition = RouteDef(**yaml.safe_load(Path(args.route).read_text(encoding="utf-8")))
    waypoints = (generate_oval(definition.shape, layout.shelves,
                               definition.shape.dock_pause)
                 if definition.shape else definition.waypoints)
    route = Route(definition, waypoints)

    print(f"Rover simulator → {args.server}")
    SimRover(route, layout, args.server, args.speed,
             args.obstacle_rate, args.drop, args.drop_clears).run()


if __name__ == "__main__":
    main()
