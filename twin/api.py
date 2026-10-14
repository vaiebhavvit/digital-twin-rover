"""HTTP + WebSocket surface.

Two groups of routes:

  /api/rover/*, /api/inspection          written by the rover
  /api/state, /api/route, /ws            read by the dashboard

Ingestion handlers are declared `def` (not `async def`) on purpose: FastAPI
runs them in a worker thread, so blocking SQLAlchemy calls never stall the
event loop that is streaming poses to the browser.
"""

from __future__ import annotations

import time

from fastapi import APIRouter, Header, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse

from . import engine, forecast
from . import route as route_mod
from .config import settings
from .db import iso_utc, repo
from .hub import hub
from .models import (InspectionIn, ObstacleIn, Pose, TelemetryIn,
                     UltrasonicIn)
from .navigation import nav

router = APIRouter()


def _auth(token: str | None):
    if settings.ingest_token and token != settings.ingest_token:
        raise HTTPException(401, "bad or missing X-Twin-Token")


# ==========================================================================
# Ingestion — the rover writes here
# ==========================================================================
@router.post("/api/rover/pose")
def post_pose(body: Pose | TelemetryIn, x_twin_token: str | None = Header(None)):
    """Rover position from encoders and IMU.

    Accepts a bare pose or {pose, telemetry}. Send it at 5–10 Hz; the twin
    streams it straight through to every open dashboard.
    """
    _auth(x_twin_token)
    if isinstance(body, Pose):
        pose, telem = body, None
    else:
        pose, telem = body.pose, body.telemetry

    if pose is None:
        fields = {k: v for k, v in (telem.model_dump() if telem else {}).items()
                  if v is not None}
        state = nav.touch(**fields)
    else:
        state = nav.update_pose(pose, telem)

    # Straying off the configured path usually means wheel slip or a route
    # that no longer matches the floor. Say so once, not every pose message.
    r = state.get("route") or {}
    if r.get("off_route_m", 0) > settings.off_route_tolerance:
        if repo.raise_alert("warning", "OFF_ROUTE", r.get("leg_to", "?"), "",
                            f"Rover is {r['off_route_m']:.2f} m off the route "
                            f"on leg {r.get('leg_from')} → {r.get('leg_to')}"):
            hub.publish_threadsafe("alerts", {"alerts": _alerts_payload()})

    hub.publish_threadsafe("pose", state)
    return {"ok": True, "route": state.get("route")}


@router.post("/api/rover/ultrasonic")
def post_ultrasonic(body: UltrasonicIn, x_twin_token: str | None = Header(None)):
    """A raw ultrasonic reading — the main obstacle interface.

    Post whatever the sensor measured. The twin compares it against the
    route's `obstacle_stop_distance`, colours the strip ahead red or green,
    and answers with the decision:

        {"clear": false, "action": "stop", "strip": "S07",
         "strip_state": "blocked", "recheck_in_s": 3.0, "attempts": 1}

    The rover should act on `action` and, when told to stop, send another
    reading after `recheck_in_s` seconds. A rover that decides for itself can
    ignore the response and simply keep reporting — the dashboard stays
    correct either way.
    """
    _auth(x_twin_token)
    decision = nav.reading(body)

    if decision["action"] == "stop":
        repo.raise_alert("critical", "OBSTACLE",
                         decision.get("strip") or "route", body.sensor,
                         decision["message"])
    else:
        repo.resolve_alerts_of_type("OBSTACLE", body.sensor)

    hub.publish_threadsafe("strips", {
        **nav.summary(), "alerts": _alerts_payload(), "pose": nav.pose,
        "decision": decision,
    })
    return decision


@router.post("/api/rover/obstacle")
def post_obstacle(body: ObstacleIn, x_twin_token: str | None = Header(None)):
    """The rover stopped for something in its path.

    Post again with `cleared: true` and the same `sensor` once the way is free.
    """
    _auth(x_twin_token)
    summary = nav.record_obstacle(body)
    strip = (summary.get("strips") or {}).get("ahead") or "route"

    if body.cleared:
        repo.resolve_alerts_of_type("OBSTACLE", body.sensor)
    else:
        d = f"{body.distance:.2f} m" if body.distance is not None else "close range"
        repo.raise_alert("critical", "OBSTACLE", strip, body.sensor,
                         f"Rover stopped — obstacle {d} ahead on {strip} "
                         f"({body.sensor} sensor)")

    payload = {**summary, "alerts": _alerts_payload(), "pose": nav.pose}
    hub.publish_threadsafe("strips", payload)
    return {"ok": True, **summary}


@router.post("/api/inspection")
def post_inspection(body: InspectionIn, x_twin_token: str | None = Header(None)):
    """One shelf scan from the vision node."""
    _auth(x_twin_token)
    result = engine.process_inspection(body)
    if not result.get("ok"):
        raise HTTPException(400, result.get("error", "inspection rejected"))
    hub.publish_threadsafe("inspection", {
        "shelf_id": result["shelf_id"],
        "results": result["results"],
        "shelves": engine.shelf_states(),
        "alerts": _alerts_payload(),
        "pose": nav.pose,
    })
    return result


@router.post("/api/rover/reset")
def post_reset(x_twin_token: str | None = Header(None)):
    """Clear the travelled path and any open obstacles."""
    _auth(x_twin_token)
    nav.reset()
    hub.publish_threadsafe("strips", {**nav.summary(), "reset": True})
    return {"ok": True}


# ==========================================================================
# Route
# ==========================================================================
@router.get("/api/route")
def get_route():
    """The patrol route, as the dashboard draws it and the rover follows it."""
    return route_mod.current_route().as_dict(repo.shelves())


@router.get("/api/strips")
def get_strips():
    """The path strips: geometry plus live state.

    Fetch this once at startup to learn the strip layout; the live states
    stream over the WebSocket after that.
    """
    if not nav.strips:
        raise HTTPException(503, "strips not built yet")
    return {"strips": nav.strips.geometry(),
            "live": nav.strips.snapshot(nav.travelled),
            "config": route_mod.current_route().definition.strips.model_dump()}


@router.post("/api/route/reload")
def reload_route():
    """Re-read config/route.yaml without restarting the server.

    Edit the YAML, call this, and the dashboard redraws the new path.
    """
    shelves = repo.shelf_defs()
    try:
        r = route_mod.load_route(shelves=shelves)
    except Exception as e:
        raise HTTPException(400, f"route file rejected: {e}")
    nav.bind(r, shelves)
    payload = r.as_dict(repo.shelves())
    payload["strips"] = nav.strips.geometry()
    hub.publish_threadsafe("route", payload)
    return {"ok": True, "name": r.name, "waypoints": len(r.waypoints),
            "strips": len(nav.strips.strips), "warnings": payload["warnings"]}


# ==========================================================================
# Dashboard reads
# ==========================================================================
def _alerts_payload():
    return [{"id": a.id, "ts": iso_utc(a.ts), "severity": a.severity,
             "atype": a.atype, "shelf_id": a.shelf_id, "product": a.product,
             "message": a.message} for a in repo.open_alerts()]


def build_snapshot() -> dict:
    shelves = engine.shelf_states()
    alerts = _alerts_payload()
    forecasts = forecast.all_forecasts()
    r = route_mod.current_route()
    pose = nav.pose or {}
    return {
        "server_time": time.time(),
        "layout": repo.get("layout_meta", {}),
        "route": r.as_dict(shelves=repo.shelves()),
        "shelves": shelves,
        "alerts": alerts,
        "forecasts": forecasts,
        "kpi": engine.kpis(shelves, alerts, forecasts, r),
        "rover": {**pose, "online": nav.is_online()},
        "strips": nav.strips.geometry() if nav.strips else [],
        "strip_state": nav.strips.snapshot(nav.travelled) if nav.strips else {},
        "obstacles": list(nav.obstacles.values()),
        "trajectory": repo.trajectory(),
        "labels": engine.LABELS,
    }


@router.get("/api/state")
def get_state():
    """Full snapshot. The WebSocket sends one on connect; this route exists so
    the dashboard still works if WebSockets are blocked."""
    return JSONResponse(build_snapshot())


@router.get("/api/history/{shelf_id}")
def get_history(shelf_id: str, limit: int = 50):
    rows = repo.shelf_history(shelf_id, limit)
    return [{"ts": iso_utc(ts), "product": p, "detected_qty": q,
             "status": st, "confidence": round(c, 2)} for ts, p, q, st, c in rows]


@router.get("/api/forecast/{product}")
def get_forecast(product: str):
    shelf = repo.home_shelf(product)
    plan = repo.planogram()
    expected = plan.get(shelf, {}).get(product, 0) if shelf else 0
    f = forecast.forecast_product(product, expected, shelf or "?")
    if not f:
        raise HTTPException(404, "not enough inspection history for a forecast")
    return f


@router.post("/api/alerts/{alert_id}/resolve")
def resolve_alert(alert_id: int):
    repo.resolve_alert(alert_id)
    hub.publish_threadsafe("alerts", {"alerts": _alerts_payload()})
    return {"ok": True}


@router.get("/api/health")
def health():
    r = route_mod.current_route()
    return {"ok": True, "rover_online": nav.is_online(),
            "route": r.name, "waypoints": len(r.waypoints),
            "strips": len(nav.strips.strips) if nav.strips else 0,
            "blocked": bool(nav.obstacles), "ws_clients": hub.client_count,
            "database": settings.database_url.split("://")[0]}


# ==========================================================================
# WebSocket
# ==========================================================================
@router.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await hub.connect(ws)
    try:
        await ws.send_json({"type": "snapshot", "payload": build_snapshot(),
                            "ts": time.time()})
        while True:
            msg = await ws.receive_text()
            if msg == "ping":
                await ws.send_text('{"type":"pong","payload":{},"ts":0}')
            elif msg == "refresh":
                await ws.send_json({"type": "snapshot", "payload": build_snapshot(),
                                    "ts": time.time()})
    except WebSocketDisconnect:
        pass
    except Exception:
        pass
    finally:
        await hub.disconnect(ws)
