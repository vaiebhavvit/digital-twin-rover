"""Path strips and the obstacle signalling state machine.

The route is cut into strips that tile the whole loop: one across the
frontage of every bay, and transit strips filling the gaps. A strip is the
unit the dashboard colours, and the unit the ultrasonic sensor talks about.

The rule the rover follows
--------------------------
    obstacle within stop_distance   ->  strip ahead RED,   rover stops
    wait recheck_interval seconds   ->  read the sensor again
    still blocked                   ->  stays RED,         keeps waiting
    clear                           ->  strip ahead GREEN, rover moves off

Two details that matter on real hardware:

*Hysteresis.* A reading only counts as clear once it exceeds
`stop_distance + clear_margin`. Without that margin an obstacle sitting right
on the threshold makes an HC-SR04 flicker between 0.39 m and 0.41 m, and the
rover lurches stop-go-stop-go. The margin turns a noisy boundary into a
decision.

*Staleness.* If the rover stops reporting — flat battery, crashed process,
Wi-Fi gone — a red strip would otherwise stay red on the dashboard forever,
which is worse than no information. Strip states expire after
`stale_after` seconds and fall back to idle.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

IDLE, CLEAR, BLOCKED, CHECKING = "idle", "clear", "blocked", "checking"


@dataclass
class Strip:
    """One fixed stretch of the path."""
    strip_id: str
    index: int
    kind: str                     # "shelf" | "transit"
    start_m: float
    end_m: float
    points: list[list[float]]     # polyline, follows the arcs
    shelf: str | None = None

    @property
    def length(self) -> float:
        return self.end_m - self.start_m

    def midpoint(self) -> list[float]:
        """The point half way along the strip, by length.

        Not points[len // 2]: a straight strip has only two points, so that
        index returns its far end and the label lands on the boundary with
        the next strip instead of over the middle of this one.
        """
        pts = self.points
        if len(pts) < 2:
            return pts[0] if pts else [0.0, 0.0]
        segs = [math.dist(pts[i - 1], pts[i]) for i in range(1, len(pts))]
        half = sum(segs) / 2.0
        run = 0.0
        for i, seg in enumerate(segs):
            if run + seg >= half:
                t = (half - run) / seg if seg else 0.0
                a, b = pts[i], pts[i + 1]
                return [round(a[0] + (b[0] - a[0]) * t, 3),
                        round(a[1] + (b[1] - a[1]) * t, 3)]
            run += seg
        return pts[-1]

    def as_dict(self) -> dict:
        return {
            "strip_id": self.strip_id, "index": self.index, "kind": self.kind,
            "shelf": self.shelf, "start_m": round(self.start_m, 2),
            "end_m": round(self.end_m, 2), "length_m": round(self.length, 2),
            "points": self.points, "label_at": self.midpoint(),
        }


@dataclass
class StripStatus:
    state: str = IDLE
    since: float = 0.0
    recheck_at: float | None = None
    attempts: int = 0
    distance: float | None = None
    sensor: str | None = None

    def as_dict(self, now: float) -> dict:
        return {
            "state": self.state,
            "since": round(now - self.since, 1) if self.since else 0.0,
            "recheck_in_s": round(max(0.0, self.recheck_at - now), 1)
                            if self.recheck_at else None,
            "attempts": self.attempts,
            "distance": round(self.distance, 2) if self.distance is not None else None,
            "sensor": self.sensor,
        }


class StripMap:
    """The strips plus their live state."""

    def __init__(self, route, shelves, config):
        self.route = route
        self.config = config
        self.strips: list[Strip] = _build(route, shelves, config)
        self.by_id = {s.strip_id: s for s in self.strips}
        self.status: dict[str, StripStatus] = {}

    # ------------------------------------------------------------------
    # Lookup
    # ------------------------------------------------------------------
    def at(self, distance_m: float) -> Strip | None:
        """The strip containing this distance along the route."""
        if not self.strips:
            return None
        d = self.route.wrap(distance_m)
        for s in self.strips:
            if s.start_m <= d < s.end_m:
                return s
        return self.strips[-1] if d >= self.strips[-1].start_m else self.strips[0]

    def ahead_of(self, distance_m: float) -> Strip | None:
        """The strip the rover is about to drive into.

        Taken `lookahead` metres in front of the rover rather than "the next
        strip in the list": an obstacle a metre away is in the stretch of path
        directly ahead, whether or not a strip boundary happens to fall
        between the rover and it.
        """
        return self.at(distance_m + self.config.lookahead)

    def status_of(self, strip_id: str) -> StripStatus:
        if strip_id not in self.status:
            self.status[strip_id] = StripStatus()
        return self.status[strip_id]

    # ------------------------------------------------------------------
    # The state machine
    # ------------------------------------------------------------------
    def report_reading(self, strip: Strip, distance: float, sensor: str,
                       stop_distance: float, clear_margin: float,
                       recheck_interval: float) -> dict:
        """Apply one ultrasonic reading and return the decision for the rover."""
        now = time.time()
        st = self.status_of(strip.strip_id)
        st.distance = distance
        st.sensor = sensor

        blocked = distance <= stop_distance
        # Hysteresis: leaving the blocked state needs a bigger gap than
        # entering it, so a reading hovering on the threshold cannot chatter.
        if st.state in (BLOCKED, CHECKING) and not blocked:
            blocked = distance < stop_distance + clear_margin

        if blocked:
            first = st.state not in (BLOCKED, CHECKING)
            if first:
                st.attempts = 1
                st.since = now
            else:
                # Only count an attempt once the previous wait has elapsed.
                if st.recheck_at is not None and now >= st.recheck_at:
                    st.attempts += 1
            st.state = BLOCKED
            st.recheck_at = now + recheck_interval
            waited = now - st.since
            return {
                "clear": False, "action": "stop",
                "strip": strip.strip_id, "strip_state": BLOCKED,
                "threshold": stop_distance,
                "recheck_in_s": recheck_interval,
                "attempts": st.attempts,
                "message": (f"Obstacle {distance:.2f} m ahead on {strip.strip_id}"
                            f"{f' ({strip.shelf})' if strip.shelf else ''} — "
                            f"stopped, rechecking in {recheck_interval:.0f} s"
                            f" (held {waited:.0f} s)"),
            }

        was_blocked = st.state in (BLOCKED, CHECKING)
        st.state = CLEAR
        st.since = now
        st.recheck_at = None
        attempts = st.attempts
        st.attempts = 0
        return {
            "clear": True, "action": "go",
            "strip": strip.strip_id, "strip_state": CLEAR,
            "threshold": stop_distance,
            "recheck_in_s": 0.0,
            "attempts": attempts,
            "message": (f"{strip.strip_id} clear at {distance:.2f} m — resuming"
                        if was_blocked
                        else f"{strip.strip_id} clear at {distance:.2f} m"),
        }

    def mark(self, strip: Strip, state: str, distance=None, sensor=None):
        """Force a strip into a state — used by the explicit obstacle endpoint."""
        st = self.status_of(strip.strip_id)
        st.state = state
        st.since = time.time()
        st.distance = distance
        st.sensor = sensor
        if state != BLOCKED:
            st.recheck_at = None
            st.attempts = 0

    def clear_all(self):
        self.status.clear()

    # ------------------------------------------------------------------
    def _expire(self, now: float):
        """Green fades back to idle; anything the rover stopped reporting is
        dropped, so the dashboard never shows a stale red."""
        hold = self.config.clear_hold
        stale = self.config.stale_after
        for sid, st in list(self.status.items()):
            age = now - st.since
            if st.state == CLEAR and age > hold:
                del self.status[sid]
            elif st.state in (BLOCKED, CHECKING) and age > stale:
                del self.status[sid]

    def snapshot(self, rover_distance: float | None = None) -> dict:
        now = time.time()
        self._expire(now)

        here = ahead = None
        if rover_distance is not None:
            s_here = self.at(rover_distance)
            s_ahead = self.ahead_of(rover_distance)
            here = s_here.strip_id if s_here else None
            ahead = s_ahead.strip_id if s_ahead else None

        states = {}
        for s in self.strips:
            st = self.status.get(s.strip_id)
            if st is None and s.strip_id not in (here, ahead):
                continue
            states[s.strip_id] = (st or StripStatus()).as_dict(now)

        blocked = [sid for sid, v in states.items() if v["state"] == BLOCKED]
        return {
            "current": here, "ahead": ahead,
            "states": states, "blocked": blocked,
            "blocked_count": len(blocked),
        }

    def geometry(self) -> list[dict]:
        return [s.as_dict() for s in self.strips]


# ==========================================================================
# Construction
# ==========================================================================
def _build(route, shelves, config) -> list[Strip]:
    """Cut the route into strips: one per bay frontage, transit in between."""
    total = route.total_length
    if total <= 0:
        return []

    by_id = {s.shelf_id: s for s in shelves}
    anchors: list[tuple[float, float, str]] = []    # (start, end, shelf_id)

    for i, wp in enumerate(route.waypoints):
        if wp.action != "inspect" or not wp.shelf:
            continue
        shelf = by_id.get(wp.shelf)
        if shelf is None:
            continue
        d = route.distance_of_index(i)
        # A bay's strip spans its frontage along the direction of travel.
        frontage = shelf.w if shelf.w >= shelf.h else shelf.h
        half = frontage / 2.0 + config.shelf_margin
        anchors.append((d - half, d + half, wp.shelf))

    anchors.sort()

    # Pull apart any overlap between neighbouring bays by meeting in the middle.
    for i in range(1, len(anchors)):
        prev_s, prev_e, prev_id = anchors[i - 1]
        s, e, sid = anchors[i]
        if s < prev_e:
            mid = (prev_e + s) / 2
            anchors[i - 1] = (prev_s, mid, prev_id)
            anchors[i] = (mid, e, sid)

    strips: list[Strip] = []
    cursor = 0.0

    def add_transit(start: float, end: float):
        span = end - start
        if span <= 0.05:
            return
        pieces = max(1, math.ceil(span / config.max_transit_length))
        step = span / pieces
        for k in range(pieces):
            a = start + k * step
            b = a + step
            strips.append(Strip(strip_id="", index=0, kind="transit",
                                start_m=a, end_m=b, points=route.slice(a, b)))

    for start, end, shelf_id in anchors:
        start = max(start, 0.0)
        end = min(end, total)
        if start > cursor:
            add_transit(cursor, start)
        strips.append(Strip(strip_id="", index=0, kind="shelf",
                            start_m=start, end_m=end,
                            points=route.slice(start, end), shelf=shelf_id))
        cursor = end

    add_transit(cursor, total)

    # Number them in route order once the list is final.
    for i, s in enumerate(strips, start=1):
        s.index = i
        s.strip_id = f"S{i:02d}"
    return strips
