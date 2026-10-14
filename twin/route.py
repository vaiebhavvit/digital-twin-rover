"""The patrol route.

The rover follows a fixed path defined in `config/route.yaml`, loaded here
once and shared by everything:

  * the dashboard draws it,
  * the simulator drives it,
  * the real rover reads it back over the API to get its waypoints,
  * and this module reports where along it the rover currently is.

One definition, four consumers. Change the YAML and all of them change.

The path is either generated from a `shape:` block (an oval track) or listed
waypoint by waypoint. Either way the rest of the system sees the same thing:
an ordered list of waypoints plus the geometry helpers below.
"""

from __future__ import annotations

import bisect
import math

import yaml

from .config import settings
from .models import RouteDef, Waypoint

INSPECT_DEFAULT_PAUSE = 2.5


def _segment_distance(px, py, ax, ay, bx, by) -> tuple[float, float]:
    """Perpendicular distance from a point to a segment, plus how far along
    the segment the closest point lies (0..1)."""
    dx, dy = bx - ax, by - ay
    length_sq = dx * dx + dy * dy
    if length_sq == 0:
        return math.hypot(px - ax, py - ay), 0.0
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy)), t


# ==========================================================================
# Oval generation
# ==========================================================================
def generate_oval(shape, shelves, dock_pause=4.0) -> list[Waypoint]:
    """Build the waypoint list for an oval track.

    Geometry: two straights at y = cy ± radius joined by semicircular arcs
    centred on the ends of the straights. Running counter-clockwise keeps the
    shelving on the rover's right all the way round, so one camera on the
    right-hand side sees every bay.

    Bays are attached to the track by their scan_x / scan_y: each one becomes
    an `inspect` waypoint at that point, facing away from the track centre.
    """
    cx, cy = shape.center
    half = shape.straight_length / 2.0
    r = shape.radius
    left_x, right_x = cx - half, cx + half
    bottom_y, top_y = cy - r, cy + r
    n = shape.corner_samples

    def arc(centre_x, start_deg, end_deg, prefix):
        """Sample an arc, excluding both endpoints (the straights own those)."""
        pts = []
        for i in range(1, n):
            t = math.radians(start_deg + (end_deg - start_deg) * i / n)
            pts.append(Waypoint(id=f"{prefix}{i}",
                                x=round(centre_x + r * math.cos(t), 3),
                                y=round(cy + r * math.sin(t), 3)))
        return pts

    # Which bays sit along which straight, in the order the rover meets them.
    def bays_on(line_y, ascending):
        out = []
        for s in shelves:
            sx = s.scan_x if s.scan_x is not None else s.x + s.w / 2
            sy = s.scan_y if s.scan_y is not None else s.y
            if abs(sy - line_y) > 0.35:
                continue
            if not (left_x - 0.01 <= sx <= right_x + 0.01):
                continue
            # Face away from the track centre: that is where the bay is.
            heading = -90.0 if s.y + s.h / 2 < cy else 90.0
            out.append((sx, Waypoint(id=s.shelf_id, x=round(sx, 3),
                                     y=round(line_y, 3), action="inspect",
                                     shelf=s.shelf_id, heading=heading)))
        out.sort(key=lambda p: p[0], reverse=not ascending)
        return [w for _, w in out]

    inspect_on = shape.inspect == "all"
    wps: list[Waypoint] = []

    # 1. bottom straight, travelling +x
    wps.append(Waypoint(id="S-BOT", x=round(left_x, 3), y=round(bottom_y, 3)))
    if inspect_on:
        wps += bays_on(bottom_y, ascending=True)
    wps.append(Waypoint(id="E-BOT", x=round(right_x, 3), y=round(bottom_y, 3)))

    # 2. right arc, -90 deg round through 0 to +90
    wps += arc(right_x, -90, 90, "R")

    # 3. top straight, travelling -x
    wps.append(Waypoint(id="S-TOP", x=round(right_x, 3), y=round(top_y, 3)))
    if inspect_on:
        wps += bays_on(top_y, ascending=False)
    wps.append(Waypoint(id="E-TOP", x=round(left_x, 3), y=round(top_y, 3)))

    # 4. left arc, +90 round through 180 to +270 (back to the bottom straight)
    left_arc = arc(left_x, 90, 270, "L")
    if shape.dock_at == "left" and left_arc:
        mid = len(left_arc) // 2
        apex = left_arc[mid]
        left_arc[mid] = Waypoint(id="DOCK", x=apex.x, y=apex.y, action="dock",
                                 heading=180.0, pause=dock_pause)
    wps += left_arc

    if shape.dock_at == "right":
        # Put the dock on the right arc instead: find its apex and relabel.
        for i, w in enumerate(wps):
            if w.id == f"R{n // 2}":
                wps[i] = Waypoint(id="DOCK", x=w.x, y=w.y, action="dock",
                                  heading=0.0, pause=dock_pause)
                break

    if shape.direction == "cw":
        # Reverse everything but keep the dock where it is; flip the headings
        # so the camera still points at the shelving.
        wps = list(reversed(wps))
        for w in wps:
            if w.heading is not None and w.action == "inspect":
                w.heading = -w.heading

    return wps


# ==========================================================================
# Route
# ==========================================================================
class Route:
    """A loaded route with pre-computed geometry."""

    def __init__(self, definition: RouteDef, waypoints: list[Waypoint] | None = None):
        self.definition = definition
        self.waypoints: list[Waypoint] = waypoints if waypoints is not None \
            else definition.waypoints
        self.loop = definition.loop
        self.default_speed = definition.default_speed

        # Leg lengths and cumulative distance, so progress is a lookup.
        pts = [(w.x, w.y) for w in self.waypoints]
        closing = pts + [pts[0]] if self.loop and len(pts) > 1 else pts
        self.legs = [math.dist(closing[i - 1], closing[i])
                     for i in range(1, len(closing))]
        self.total_length = sum(self.legs)

        self.cumulative = [0.0]
        for leg in self.legs:
            self.cumulative.append(self.cumulative[-1] + leg)

    # ------------------------------------------------------------------
    @property
    def name(self) -> str:
        return self.definition.name

    @property
    def inspect_stops(self) -> list[Waypoint]:
        return [w for w in self.waypoints if w.action == "inspect"]

    @property
    def stop_distance(self) -> float:
        return self.definition.obstacle_stop_distance

    @property
    def recheck_interval(self) -> float:
        return self.definition.recheck_interval

    def shelf_waypoint(self, shelf_id: str) -> Waypoint | None:
        return next((w for w in self.waypoints
                     if w.action == "inspect" and w.shelf == shelf_id), None)

    def pause_for(self, wp: Waypoint) -> float:
        if wp.pause is not None:
            return wp.pause
        return INSPECT_DEFAULT_PAUSE if wp.action == "inspect" else 0.0

    def speed_for(self, index: int) -> float:
        wp = self.waypoints[index % len(self.waypoints)]
        return wp.speed or self.default_speed

    def distance_of_index(self, i: int) -> float:
        return self.cumulative[min(i, len(self.cumulative) - 1)]

    # ------------------------------------------------------------------
    # Geometry along the path
    # ------------------------------------------------------------------
    def wrap(self, d: float) -> float:
        if not self.total_length:
            return 0.0
        return d % self.total_length if self.loop \
            else max(0.0, min(d, self.total_length))

    def point_at(self, d: float) -> tuple[float, float]:
        """The point `d` metres along the route from waypoint 0."""
        if not self.legs:
            w = self.waypoints[0]
            return w.x, w.y
        d = self.wrap(d)
        i = max(0, bisect.bisect_right(self.cumulative, d) - 1)
        i = min(i, len(self.legs) - 1)
        leg = self.legs[i] or 1.0
        t = (d - self.cumulative[i]) / leg
        n = len(self.waypoints)
        a, b = self.waypoints[i], self.waypoints[(i + 1) % n]
        return a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t

    def slice(self, start_m: float, end_m: float) -> list[list[float]]:
        """Polyline of the route between two distances, corners included.

        Used to draw a strip: on a straight this is two points, on an arc it
        picks up every sampled waypoint in between so the strip curves.
        """
        pts = [list(self.point_at(start_m))]
        n = len(self.waypoints)
        for i in range(1, len(self.cumulative)):
            c = self.cumulative[i]
            if start_m < c < end_m:
                w = self.waypoints[i % n]
                pts.append([w.x, w.y])
        pts.append(list(self.point_at(end_m)))
        return [[round(p[0], 3), round(p[1], 3)] for p in pts]

    # ------------------------------------------------------------------
    def locate(self, x: float, y: float) -> dict:
        """Where is this pose along the route?

        Returns the leg the rover is on, how far round the lap it has got, and
        which inspection stop is coming next. Also reports how far the rover
        has strayed from the path, which is what tells you a wheel is slipping
        or the route no longer matches the floor.
        """
        if not self.legs:
            return {"progress_pct": 0.0, "travelled_m": 0.0, "leg": 0,
                    "off_route_m": 0.0, "next_stop": None,
                    "distance_to_next_m": 0.0}

        best_leg, best_t, best_d = 0, 0.0, float("inf")
        n = len(self.waypoints)
        for i in range(len(self.legs)):
            a = self.waypoints[i]
            b = self.waypoints[(i + 1) % n]
            d, t = _segment_distance(x, y, a.x, a.y, b.x, b.y)
            if d < best_d:
                best_leg, best_t, best_d = i, t, d

        travelled = self.cumulative[best_leg] + best_t * self.legs[best_leg]

        # The next waypoint whose action is an inspection.
        next_stop, distance_to_next = None, 0.0
        for step in range(1, len(self.legs) + 1):
            idx = (best_leg + step) % n
            wp = self.waypoints[idx]
            if wp.action == "inspect":
                next_stop = wp
                target = self.cumulative[best_leg + step] \
                    if best_leg + step < len(self.cumulative) \
                    else self.total_length + self.cumulative[(best_leg + step) % n]
                distance_to_next = max(0.0, target - travelled)
                break

        return {
            "progress_pct": round(100 * travelled / self.total_length, 1),
            "travelled_m": round(travelled, 2),
            "total_length_m": round(self.total_length, 2),
            "leg": best_leg,
            "leg_from": self.waypoints[best_leg].id,
            "leg_to": self.waypoints[(best_leg + 1) % n].id,
            "off_route_m": round(best_d, 3),
            "next_stop": next_stop.id if next_stop else None,
            "next_shelf": next_stop.shelf if next_stop else None,
            "distance_to_next_m": round(distance_to_next, 2),
            "eta_next_s": round(distance_to_next / max(self.default_speed, 0.01), 1),
        }

    # ------------------------------------------------------------------
    def validate(self, shelves) -> list[str]:
        """Sanity-check the route against the warehouse layout.

        A route is easy to get subtly wrong — a waypoint inside a rack, a leg
        that cuts through one, an `inspect` pointing at a bay that does not
        exist. These warnings are returned to the dashboard and shown on
        screen rather than discovered on the warehouse floor.
        """
        warnings: list[str] = []
        known = {s.shelf_id: s for s in shelves}

        for wp in self.waypoints:
            if wp.action == "inspect":
                if not wp.shelf:
                    warnings.append(f"{wp.id}: action is inspect but no shelf given")
                elif wp.shelf not in known:
                    warnings.append(f"{wp.id}: inspects unknown bay '{wp.shelf}'")
            for s in known.values():
                if s.x <= wp.x <= s.x + s.w and s.y <= wp.y <= s.y + s.h:
                    warnings.append(f"{wp.id} sits inside bay {s.shelf_id}")

        n = len(self.waypoints)
        span = n if self.loop else n - 1
        for i in range(span):
            a, b = self.waypoints[i], self.waypoints[(i + 1) % n]
            for s in known.values():
                if _segment_hits_rect(a.x, a.y, b.x, b.y, s.x, s.y, s.w, s.h):
                    warnings.append(f"leg {a.id} → {b.id} crosses bay {s.shelf_id}")

        inspected = {w.shelf for w in self.inspect_stops if w.shelf}
        for shelf_id in known:
            if shelf_id not in inspected:
                warnings.append(f"bay {shelf_id} is never inspected by this route")

        return warnings

    def as_dict(self, shelves=None) -> dict:
        d = self.definition
        return {
            "name": self.name,
            "loop": self.loop,
            "generated": d.shape is not None,
            "default_speed": self.default_speed,
            "obstacle_stop_distance": d.obstacle_stop_distance,
            "recheck_interval": d.recheck_interval,
            "clear_margin": d.clear_margin,
            "total_length_m": round(self.total_length, 2),
            "inspect_stops": len(self.inspect_stops),
            "waypoints": [w.model_dump() for w in self.waypoints],
            "warnings": self.validate(shelves) if shelves is not None else [],
        }


def _segment_hits_rect(x1, y1, x2, y2, rx, ry, rw, rh) -> bool:
    """Does the segment cross the rectangle's interior?"""
    if (x1 < rx and x2 < rx) or (x1 > rx + rw and x2 > rx + rw):
        return False
    if (y1 < ry and y2 < ry) or (y1 > ry + rh and y2 > ry + rh):
        return False
    edges = [(rx, ry, rx + rw, ry), (rx + rw, ry, rx + rw, ry + rh),
             (rx + rw, ry + rh, rx, ry + rh), (rx, ry + rh, rx, ry)]
    return any(_segments_cross(x1, y1, x2, y2, *e) for e in edges)


def _segments_cross(ax, ay, bx, by, cx, cy, dx, dy) -> bool:
    def side(px, py, qx, qy, rx, ry):
        v = (qx - px) * (ry - py) - (qy - py) * (rx - px)
        return 0 if abs(v) < 1e-12 else (1 if v > 0 else -1)
    d1, d2 = side(ax, ay, bx, by, cx, cy), side(ax, ay, bx, by, dx, dy)
    d3, d4 = side(cx, cy, dx, dy, ax, ay), side(cx, cy, dx, dy, bx, by)
    return d1 != d2 and d3 != d4


# --------------------------------------------------------------------------
_route: Route | None = None


def load_route(path=None, shelves=None) -> Route:
    """Read config/route.yaml and make it the active route."""
    global _route
    path = path or settings.route_file
    if not path.exists():
        raise FileNotFoundError(f"route file not found: {path}")
    definition = RouteDef(**yaml.safe_load(path.read_text(encoding="utf-8")))

    if definition.shape is not None:
        if not shelves:
            raise ValueError("a generated route needs the warehouse layout; "
                             "load the layout before the route")
        waypoints = generate_oval(definition.shape, shelves,
                                  definition.shape.dock_pause)
    else:
        waypoints = definition.waypoints

    if len(waypoints) < 2:
        raise ValueError("a route needs at least two waypoints")

    _route = Route(definition, waypoints)
    return _route


def current_route() -> Route:
    if _route is None:
        raise RuntimeError("no route loaded yet")
    return _route
