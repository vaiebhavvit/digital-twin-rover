"""The twin itself: expected state vs observed state, and what follows.

This is where the project's novelty lives. The planogram is the expected
state of the warehouse; the rover reports observed state; the engine
reconciles the two and emits alerts and inventory status.
"""

from __future__ import annotations

import math

from .config import settings
from .db import iso_utc, repo
from .navigation import nav

OK, LOW_STOCK, OUT_OF_STOCK, MISPLACED, UNSCANNED = (
    "OK", "LOW_STOCK", "OUT_OF_STOCK", "MISPLACED", "UNSCANNED")

SEVERITY_RANK = {UNSCANNED: 0, OK: 1, LOW_STOCK: 2, OUT_OF_STOCK: 3, MISPLACED: 4}

LABELS = {OK: "In spec", LOW_STOCK: "Low stock", OUT_OF_STOCK: "Empty",
          MISPLACED: "Misplaced", UNSCANNED: "Not scanned"}


def classify(qty: int, expected: int) -> str:
    if qty <= 0:
        return OUT_OF_STOCK
    if expected and qty < settings.low_stock_ratio * expected:
        return LOW_STOCK
    return OK


# --------------------------------------------------------------------------
# Shelf association
# --------------------------------------------------------------------------
def associate_shelf(pose) -> str | None:
    """Pick the shelf a scan belongs to when the vision node did not say.

    Scores every shelf by distance from the rover and by how closely it sits
    in the direction the camera is pointing, so the rover does not credit a
    scan to the bay behind it.
    """
    if pose is None:
        return None
    best, best_score = None, None
    for sh in repo.shelves():
        cx, cy = sh.x + sh.w / 2, sh.y + sh.h / 2
        dx, dy = cx - pose.x, cy - pose.y
        dist = math.hypot(dx, dy)
        if dist > settings.scan_association_radius:
            continue
        # cos of the angle between heading and the shelf direction: 1 = ahead
        facing = math.cos(math.atan2(dy, dx) - pose.theta)
        if facing < 0.2:              # shelf is beside or behind the rover
            continue
        score = dist * (2.0 - facing)
        if best_score is None or score < best_score:
            best, best_score = sh.shelf_id, score
    return best


# --------------------------------------------------------------------------
# Inspection processing
# --------------------------------------------------------------------------
def process_inspection(payload) -> dict:
    """Apply one shelf scan to the twin. Returns a summary for broadcasting."""
    shelf_id = payload.shelf_id or associate_shelf(payload.pose)
    if not shelf_id:
        return {"ok": False, "error": "no shelf_id given and none matched the pose"}

    known = {s.shelf_id for s in repo.shelves()}
    if shelf_id not in known:
        return {"ok": False, "error": f"unknown shelf '{shelf_id}'"}

    expected = repo.planogram().get(shelf_id, {})
    px = payload.pose.x if payload.pose else None
    py = payload.pose.y if payload.pose else None

    rows, results, keep, new_alerts = [], [], set(), []
    seen = set()

    for det in payload.detections:
        seen.add(det.product)
        if det.product in expected:
            status = classify(det.qty, expected[det.product])
            if status == OUT_OF_STOCK:
                keep.add((OUT_OF_STOCK, det.product))
                if repo.raise_alert("critical", OUT_OF_STOCK, shelf_id, det.product,
                                    f"{det.product} shelf empty on {shelf_id}"):
                    new_alerts.append(det.product)
            elif status == LOW_STOCK:
                keep.add((LOW_STOCK, det.product))
                if repo.raise_alert(
                        "warning", LOW_STOCK, shelf_id, det.product,
                        f"{det.product} low on {shelf_id} — {det.qty} of "
                        f"{expected[det.product]} units"):
                    new_alerts.append(det.product)
        else:
            # Shelf-compliance violation: right product, wrong bay.
            status = MISPLACED
            home = repo.home_shelf(det.product)
            msg = (f"{det.product} found on {shelf_id}, belongs on {home}"
                   if home else f"Unregistered item {det.product} on {shelf_id}")
            keep.add((MISPLACED, det.product))
            if repo.raise_alert("critical", MISPLACED, shelf_id, det.product, msg):
                new_alerts.append(det.product)

        rows.append({"shelf_id": shelf_id, "product": det.product,
                     "detected_qty": det.qty, "confidence": det.confidence,
                     "status": status, "pose_x": px, "pose_y": py})
        results.append({"product": det.product, "qty": det.qty, "status": status})

    # Planogram products the camera did not see at all.
    for product, exp_qty in expected.items():
        if product in seen:
            continue
        keep.add((OUT_OF_STOCK, product))
        if repo.raise_alert("critical", OUT_OF_STOCK, shelf_id, product,
                            f"{product} not detected on {shelf_id}"):
            new_alerts.append(product)
        rows.append({"shelf_id": shelf_id, "product": product, "detected_qty": 0,
                     "confidence": 0.0, "status": OUT_OF_STOCK,
                     "pose_x": px, "pose_y": py})
        results.append({"product": product, "qty": 0, "status": OUT_OF_STOCK})

    repo.add_inspections(rows)
    resolved = repo.resolve_stale(shelf_id, keep)

    if payload.pose:
        nav.update_pose(payload.pose, payload.telemetry)
    nav.touch(current_shelf=shelf_id, state="inspecting")

    return {"ok": True, "shelf_id": shelf_id, "results": results,
            "new_alerts": len(new_alerts), "resolved": resolved}


# --------------------------------------------------------------------------
# Dashboard state
# --------------------------------------------------------------------------
def shelf_states() -> list[dict]:
    plan = repo.planogram()
    latest = repo.latest_by_shelf()
    misplaced_by_shelf: dict[str, list[str]] = {}
    for a in repo.open_alerts(200):
        if a.atype == MISPLACED:
            misplaced_by_shelf.setdefault(a.shelf_id, []).append(a.product)

    out = []
    for sh in repo.shelves():
        items, worst, last_ts = [], UNSCANNED, None
        for product, exp_qty in plan.get(sh.shelf_id, {}).items():
            insp = latest.get((sh.shelf_id, product))
            if insp:
                last_ts = max(last_ts or insp.ts, insp.ts)
                items.append({"product": product, "expected": exp_qty,
                              "detected": insp.detected_qty, "status": insp.status,
                              "confidence": round(insp.confidence, 2)})
                if SEVERITY_RANK[insp.status] > SEVERITY_RANK[worst]:
                    worst = insp.status
            else:
                items.append({"product": product, "expected": exp_qty,
                              "detected": None, "status": UNSCANNED,
                              "confidence": None})

        for stray in misplaced_by_shelf.get(sh.shelf_id, []):
            worst = MISPLACED
            items.append({"product": stray, "expected": 0, "detected": 1,
                          "status": MISPLACED, "confidence": None})

        out.append({
            "shelf_id": sh.shelf_id, "zone": sh.zone,
            "x": sh.x, "y": sh.y, "w": sh.w, "h": sh.h,
            "scan_x": sh.scan_x, "scan_y": sh.scan_y,
            "status": worst, "items": items,
            "last_scan": iso_utc(last_ts),
        })
    return out


def kpis(shelves, alerts, forecasts, route=None) -> dict:
    ok = sum(1 for s in shelves if s["status"] == OK)
    total = len(shelves) or 1
    progress = ((nav.pose or {}).get("route") or {})
    return {
        "shelves_total": len(shelves),
        "shelves_ok": ok,
        "compliance_pct": round(100 * ok / total),
        "open_alerts": len(alerts),
        "critical_alerts": sum(1 for a in alerts if a["severity"] == "critical"),
        "total_scans": repo.count_inspections(),
        "restock_now": sum(1 for f in forecasts
                           if f["priority"] in ("Restock now", "Critical")),
        "route_progress_pct": progress.get("progress_pct", 0.0),
        "route_length_m": route.total_length if route else 0.0,
        "inspect_stops": len(route.inspect_stops) if route else 0,
        "bays_covered": len({w.shelf for w in route.inspect_stops if w.shelf})
                        if route else 0,
        "obstacles_open": len(nav.obstacles),
    }
