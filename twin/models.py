"""The integration contract.

Everything the rover sends to the twin is validated against these models. If a
payload validates, the twin accepts it — this is the whole coupling surface
between the rover firmware and this dashboard.

Conventions:
  * Distances in metres, angles in degrees in the YAML (easier to write by
    hand) and radians on the wire (easier to compute with).
  * Positions are in the warehouse frame defined by config/layout.yaml, with
    the origin at the floor plan's bottom-left corner.
"""

from __future__ import annotations

import math
from typing import Any, Literal

from pydantic import BaseModel, Field, field_validator

NavState = Literal["idle", "moving", "inspecting", "blocked", "docked",
                   "charging", "stopped"]

StripState = Literal["idle", "clear", "blocked", "checking"]


# --------------------------------------------------------------------------
# Route (config/route.yaml)
# --------------------------------------------------------------------------
class Waypoint(BaseModel):
    id: str
    x: float
    y: float
    action: Literal["move", "inspect", "dock", "wait"] = "move"
    shelf: str | None = None
    heading: float | None = None      # degrees; 0 = +x, 90 = +y
    pause: float | None = None        # seconds held at this waypoint
    speed: float | None = None        # m/s for the leg into this waypoint

    @property
    def heading_rad(self) -> float | None:
        return math.radians(self.heading) if self.heading is not None else None


class OvalShape(BaseModel):
    """Generates an oval track: two straights joined by two arcs."""
    type: Literal["oval"] = "oval"
    center: list[float] = [12.0, 6.5]
    straight_length: float = Field(default=14.0, gt=0)
    radius: float = Field(default=3.5, gt=0)
    corner_samples: int = Field(default=12, ge=3, le=64)
    direction: Literal["ccw", "cw"] = "ccw"
    dock_at: Literal["left", "right", "none"] = "left"
    dock_pause: float = 4.0
    inspect: Literal["all", "none"] = "all"


class StripConfig(BaseModel):
    shelf_margin: float = Field(default=0.2, ge=0)
    max_transit_length: float = Field(default=3.0, gt=0.2)
    lookahead: float = Field(default=0.8, ge=0)
    clear_hold: float = Field(default=4.0, ge=0)
    stale_after: float = Field(default=30.0, gt=0)


class RouteDef(BaseModel):
    name: str = "Patrol route"
    loop: bool = True
    default_speed: float = Field(default=0.45, gt=0)
    obstacle_stop_distance: float = Field(default=0.4, gt=0)
    recheck_interval: float = Field(default=3.0, gt=0)
    clear_margin: float = Field(default=0.1, ge=0)
    shape: OvalShape | None = None
    strips: StripConfig = StripConfig()
    waypoints: list[Waypoint] = []

    @field_validator("waypoints")
    @classmethod
    def unique_ids(cls, v):
        ids = [w.id for w in v]
        dupes = {i for i in ids if ids.count(i) > 1}
        if dupes:
            raise ValueError(f"duplicate waypoint ids: {sorted(dupes)}")
        return v


# --------------------------------------------------------------------------
# Rover telemetry
# --------------------------------------------------------------------------
class Pose(BaseModel):
    """Where the rover is, from wheel encoders plus the IMU."""
    x: float
    y: float
    theta: float = 0.0                # radians, CCW from +x
    stamp: float | None = None


class Telemetry(BaseModel):
    battery: float | None = None
    state: NavState | None = None
    lap: int | None = None
    speed: float | None = None        # m/s
    waypoint: str | None = None       # id of the waypoint being driven to
    target_shelf: str | None = None


class TelemetryIn(BaseModel):
    pose: Pose | None = None
    telemetry: Telemetry | None = None


class UltrasonicIn(BaseModel):
    """A raw ultrasonic reading — the primary obstacle interface.

    The rover posts whatever the sensor measured; the twin compares it against
    `obstacle_stop_distance`, colours the strip ahead, and answers with the
    decision. See the response model below.
    """
    distance: float = Field(ge=0, description="metres to the nearest echo")
    sensor: str = "front"
    pose: Pose | None = None
    # Set true when the rover already stopped itself, so the twin does not
    # have to infer it from the nav state.
    stopped: bool = False
    stamp: float | None = None


class UltrasonicDecision(BaseModel):
    """What the twin tells the rover to do about that reading."""
    clear: bool
    action: Literal["go", "stop"]
    strip: str | None = None
    strip_state: StripState = "idle"
    threshold: float = 0.4
    recheck_in_s: float = 0.0
    attempts: int = 0
    message: str = ""


class ObstacleIn(BaseModel):
    """An explicit obstacle report, for a rover that decides for itself.

    Prefer /api/rover/ultrasonic. This endpoint stays for bumper switches and
    for firmware that only reports a boolean.
    """
    x: float | None = None
    y: float | None = None
    distance: float | None = None
    sensor: str = "front"
    cleared: bool = False
    pose: Pose | None = None
    stamp: float | None = None


# --------------------------------------------------------------------------
# Inspection
# --------------------------------------------------------------------------
class Detection(BaseModel):
    product: str
    qty: int = Field(ge=0)
    confidence: float = Field(default=0.9, ge=0, le=1)
    bbox: list[float] | None = None    # [x, y, w, h] in image pixels


class InspectionIn(BaseModel):
    """One shelf scan.

    `shelf_id` is optional: if the vision node does not know bay names, omit it
    and send `pose` — the twin matches the scan to the nearest bay the camera
    is facing.
    """
    shelf_id: str | None = None
    detections: list[Detection] = []
    pose: Pose | None = None
    telemetry: Telemetry | None = None
    stamp: float | None = None


# --------------------------------------------------------------------------
# Layout (config/layout.yaml)
# --------------------------------------------------------------------------
class ShelfDef(BaseModel):
    shelf_id: str
    zone: str = "A"
    x: float            # south-west corner, metres
    y: float
    w: float
    h: float
    scan_x: float | None = None
    scan_y: float | None = None
    products: dict[str, int] = {}     # product name -> expected quantity


class LayoutDef(BaseModel):
    name: str = "Warehouse"
    frame_id: str = "map"
    bounds: list[float] = [0, 0, 24, 13]      # [minx, miny, maxx, maxy]
    dock: list[float] = [1.5, 6.5]
    obstacles: list[list[float]] = []          # fixed [x, y, w, h] blocks
    shelves: list[ShelfDef] = []


# --------------------------------------------------------------------------
# Outbound
# --------------------------------------------------------------------------
class Envelope(BaseModel):
    """Every WebSocket frame the dashboard receives."""
    type: Literal["snapshot", "pose", "inspection", "alerts", "strips", "route"]
    payload: dict[str, Any]
    ts: float
