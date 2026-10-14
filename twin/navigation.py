"""Live rover state: pose, route progress, path strips, obstacles.

The rover knows where it is from wheel encoders and the IMU; this module
keeps that pose, works out where along the configured route it sits, which
strip it is on and which strip is ahead of it, and runs the obstacle
signalling through `StripMap`.

Pose is held in memory because it changes ten times a second and nobody
queries history at that rate. The travelled path is persisted, but only once
the rover has moved `DT_TRAJ_STEP` metres — enough to draw the path, without
writing 36,000 rows an hour.
"""

from __future__ import annotations

import math
import time

from .config import settings
from .db import repo
from .route import current_route
from .strips import BLOCKED, CLEAR, IDLE, StripMap


class Navigator:
    def __init__(self):
        self.pose: dict | None = None
        self.last_persisted_xy: tuple[float, float] | None = None
        self.obstacles: dict[str, dict] = {}       # sensor -> active obstacle
        self.strips: StripMap | None = None
        self._restore()

    # ------------------------------------------------------------------
    def bind(self, route, shelves):
        """Build the strip map. Called at startup and on every route reload."""
        self.strips = StripMap(route, shelves, route.definition.strips)

    def _restore(self):
        """Survive a server restart without losing the demo."""
        self.pose = repo.get("rover_pose")
        for o in (repo.get("active_obstacles") or []):
            self.obstacles[o["sensor"]] = o

    # ------------------------------------------------------------------
    @property
    def travelled(self) -> float | None:
        if not self.pose:
            return None
        return (self.pose.get("route") or {}).get("travelled_m")

    def update_pose(self, pose, telemetry=None) -> dict:
        prev = self.pose or {}
        state = {
            "x": pose.x, "y": pose.y, "theta": pose.theta,
            "ts": time.time(),
            "battery": prev.get("battery"),
            "state": prev.get("state", "moving"),
            "lap": prev.get("lap", 0),
            "speed": prev.get("speed"),
            "waypoint": prev.get("waypoint"),
            "target_shelf": prev.get("target_shelf"),
            "current_shelf": prev.get("current_shelf"),
        }
        if telemetry:
            for k in ("battery", "state", "lap", "speed", "waypoint", "target_shelf"):
                v = getattr(telemetry, k, None)
                if v is not None:
                    state[k] = v

        route = current_route()
        state["route"] = route.locate(pose.x, pose.y)

        if self.strips:
            d = state["route"]["travelled_m"]
            here = self.strips.at(d)
            ahead = self.strips.ahead_of(d)
            state["strip"] = here.strip_id if here else None
            state["strip_ahead"] = ahead.strip_id if ahead else None

        self.pose = state

        last = self.last_persisted_xy
        if last is None or math.dist(last, (pose.x, pose.y)) >= settings.trajectory_min_step:
            repo.add_trajectory(pose.x, pose.y, pose.theta, state.get("state", "moving"))
            self.last_persisted_xy = (pose.x, pose.y)
            repo.put("rover_pose", state)
        return state

    def touch(self, **kw) -> dict:
        """Update telemetry fields without a new pose."""
        self.pose = {**(self.pose or {"x": 0, "y": 0, "theta": 0}),
                     **kw, "ts": time.time()}
        return self.pose

    def is_online(self) -> bool:
        return bool(self.pose) and (time.time() - self.pose.get("ts", 0)) < settings.rover_timeout_s

    # ------------------------------------------------------------------
    # Ultrasonic — the primary obstacle path
    # ------------------------------------------------------------------
    def reading(self, msg) -> dict:
        """Apply one ultrasonic reading and return the rover's instruction."""
        route = current_route()
        if msg.pose:
            self.update_pose(msg.pose)

        if not self.strips:
            return {"clear": True, "action": "go", "strip": None,
                    "strip_state": IDLE, "threshold": route.stop_distance,
                    "recheck_in_s": 0.0, "attempts": 0,
                    "message": "no strips configured"}

        d = self.travelled
        if d is None:
            return {"clear": True, "action": "go", "strip": None,
                    "strip_state": IDLE, "threshold": route.stop_distance,
                    "recheck_in_s": 0.0, "attempts": 0,
                    "message": "no pose yet — send a pose before readings"}

        strip = self.strips.ahead_of(d)
        decision = self.strips.report_reading(
            strip, msg.distance, msg.sensor,
            route.stop_distance, route.definition.clear_margin,
            route.recheck_interval)

        if decision["clear"]:
            self.obstacles.pop(msg.sensor, None)
            if self.pose and self.pose.get("state") == "blocked":
                self.touch(state="moving")
        else:
            th = self.pose.get("theta", 0.0) if self.pose else 0.0
            px = self.pose.get("x", 0.0) if self.pose else 0.0
            py = self.pose.get("y", 0.0) if self.pose else 0.0
            self.obstacles[msg.sensor] = {
                "sensor": msg.sensor,
                "x": round(px + math.cos(th) * msg.distance, 3),
                "y": round(py + math.sin(th) * msg.distance, 3),
                "distance": msg.distance, "ts": time.time(),
                "strip": strip.strip_id, "shelf": strip.shelf,
                "attempts": decision["attempts"],
            }
            self.touch(state="blocked")

        repo.put("active_obstacles", list(self.obstacles.values()))
        return decision

    # ------------------------------------------------------------------
    # Explicit obstacle reports (bumpers, boolean-only firmware)
    # ------------------------------------------------------------------
    def record_obstacle(self, msg) -> dict:
        if msg.pose:
            self.update_pose(msg.pose)

        strip = None
        d = self.travelled
        if self.strips and d is not None:
            strip = self.strips.ahead_of(d)

        if msg.cleared:
            self.obstacles.pop(msg.sensor, None)
            if strip:
                self.strips.mark(strip, CLEAR)
            if self.pose and self.pose.get("state") == "blocked":
                self.touch(state="moving")
        else:
            px = self.pose.get("x", 0.0) if self.pose else 0.0
            py = self.pose.get("y", 0.0) if self.pose else 0.0
            th = self.pose.get("theta", 0.0) if self.pose else 0.0
            dist = msg.distance if msg.distance is not None else 0.3
            ox = msg.x if msg.x is not None else px + math.cos(th) * dist
            oy = msg.y if msg.y is not None else py + math.sin(th) * dist
            self.obstacles[msg.sensor] = {
                "sensor": msg.sensor, "x": round(ox, 3), "y": round(oy, 3),
                "distance": dist, "ts": time.time(),
                "strip": strip.strip_id if strip else None,
                "shelf": strip.shelf if strip else None, "attempts": 1,
            }
            if strip:
                self.strips.mark(strip, BLOCKED, dist, msg.sensor)
            self.touch(state="blocked")

        repo.put("active_obstacles", list(self.obstacles.values()))
        return self.summary()

    # ------------------------------------------------------------------
    def summary(self) -> dict:
        out = {"obstacles": list(self.obstacles.values()),
               "blocked": bool(self.obstacles)}
        if self.strips:
            out["strips"] = self.strips.snapshot(self.travelled)
        return out

    def reset(self):
        self.pose = None
        self.obstacles.clear()
        self.last_persisted_xy = None
        if self.strips:
            self.strips.clear_all()
        repo.clear_trajectory()
        repo.put("active_obstacles", [])


nav = Navigator()
