"""Integration check for the ingestion contract.

Exercises every way the rover can talk to the twin, so you can prove the
interface works before the hardware is ready — and re-run it after any change
to the route or the code.

    python -m tools.test_integration
    python -m tools.test_integration --server http://192.168.1.50:8000
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import requests

PASS, FAIL = "  ok  ", " FAIL "
results: list[tuple[bool, str]] = []


def check(name: str, condition: bool, detail: str = ""):
    results.append((bool(condition), name))
    print(f"[{PASS if condition else FAIL}] {name}" + (f" — {detail}" if detail else ""))


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
    args = ap.parse_args()
    base = args.server.rstrip("/")
    s = requests.Session()

    # ---- 0. the server is there ----------------------------------------
    r = s.get(f"{base}/api/health", timeout=5)
    check("server reachable", r.status_code == 200, r.text[:70])

    state = s.get(f"{base}/api/state", timeout=10).json()
    shelves = state["shelves"]
    check("layout loaded", len(shelves) > 0, f"{len(shelves)} bays")

    # ---- 1. the route ---------------------------------------------------
    route = s.get(f"{base}/api/route", timeout=5).json()
    wps = route["waypoints"]
    check("route served to the rover", len(wps) >= 2,
          f"{len(wps)} waypoints, {route['total_length_m']} m")
    check("route has inspection stops", route["inspect_stops"] > 0,
          f"{route['inspect_stops']} stops")

    stops = [w for w in wps if w["action"] == "inspect"]
    covered = {w["shelf"] for w in stops}
    check("every bay is on the route",
          covered >= {sh["shelf_id"] for sh in shelves},
          f"{len(covered)} of {len(shelves)} bays")
    check("route passes its own validation", not route["warnings"],
          "; ".join(route["warnings"][:2]) or "no warnings")

    # ---- 2. strips -------------------------------------------------------
    sdata = s.get(f"{base}/api/strips", timeout=5).json()
    strips = sdata["strips"]
    check("route cut into strips", len(strips) > 0, f"{len(strips)} strips")

    shelf_strips = [x for x in strips if x["kind"] == "shelf"]
    check("every bay has a strip at its frontage",
          {x["shelf"] for x in shelf_strips} == {sh["shelf_id"] for sh in shelves},
          f"{len(shelf_strips)} bay strips")

    covered = sum(x["length_m"] for x in strips)
    check("strips tile the whole loop with no gaps",
          abs(covered - route["total_length_m"]) < 0.05,
          f"{covered:.2f} of {route['total_length_m']} m")

    ordered = all(strips[i]["start_m"] >= strips[i - 1]["end_m"] - 1e-6
                  for i in range(1, len(strips)))
    check("strips do not overlap", ordered)

    # A strip on an arc picks up every sampled waypoint it spans, so it has
    # noticeably more points than one lying along a straight.
    curved = [x for x in strips if len(x["points"]) >= 5]
    check("strips on the arcs are drawn curved, not chorded",
          len(curved) > 0, f"{len(curved)} of {len(strips)} strips bend")

    # ---- 3. hot reload ---------------------------------------------------
    r = s.post(f"{base}/api/route/reload", timeout=10)
    check("route reloads without a restart", r.status_code == 200, r.text[:70])

    # ---- 3. pose and route progress -------------------------------------
    first = stops[0]
    r = s.post(f"{base}/api/rover/pose", json={
        "pose": {"x": first["x"], "y": first["y"],
                 "theta": math.radians(first.get("heading") or 90)},
        "telemetry": {"battery": 88, "state": "moving", "lap": 1}}, timeout=5)
    check("pose ingest", r.status_code == 200, r.text[:60])

    prog = r.json().get("route") or {}
    check("progress computed along the route",
          "progress_pct" in prog and prog["off_route_m"] < 0.05,
          f"{prog.get('progress_pct')}% of the lap, "
          f"{prog.get('off_route_m')} m off route")
    check("next inspection stop identified", prog.get("next_stop") is not None,
          f"next {prog.get('next_stop')} in {prog.get('distance_to_next_m')} m")

    # ---- 4. straying off the route is noticed ---------------------------
    s.post(f"{base}/api/rover/pose", json={
        "pose": {"x": first["x"], "y": first["y"] - 2.2, "theta": 0}}, timeout=5)
    alerts = s.get(f"{base}/api/state", timeout=10).json()["alerts"]
    check("off-route drift raises an alert",
          any(a["atype"] == "OFF_ROUTE" for a in alerts),
          next((a["message"] for a in alerts if a["atype"] == "OFF_ROUTE"), "none"))
    s.post(f"{base}/api/rover/pose", json={
        "pose": {"x": first["x"], "y": first["y"], "theta": 0}}, timeout=5)

    # ---- 5. the ultrasonic cycle: clear -> blocked -> recheck -> clear ----
    threshold = route["obstacle_stop_distance"]
    margin = route.get("clear_margin", 0.1)
    recheck = route["recheck_interval"]

    r = s.post(f"{base}/api/rover/ultrasonic",
               json={"distance": 2.0, "sensor": "front"}, timeout=5).json()
    check("clear reading -> GO", r["action"] == "go" and r["clear"],
          f"{r['strip']} {r['strip_state']}")
    green_strip = r["strip"]

    r = s.post(f"{base}/api/rover/ultrasonic",
               json={"distance": round(threshold - 0.1, 2), "sensor": "front"},
               timeout=5).json()
    check("close reading -> STOP", r["action"] == "stop" and not r["clear"],
          f"{r['strip']} {r['strip_state']}")
    check("the strip AHEAD is the one that turns red",
          r["strip"] == green_strip, f"{r['strip']} vs {green_strip}")
    check("twin tells the rover when to look again",
          abs(r["recheck_in_s"] - recheck) < 0.01, f"{r['recheck_in_s']} s")

    live = s.get(f"{base}/api/strips", timeout=5).json()["live"]
    check("blocked strip is published to the dashboard",
          r["strip"] in live["blocked"], f"blocked: {live['blocked']}")

    alerts = s.get(f"{base}/api/state", timeout=10).json()["alerts"]
    check("obstacle raises an alert",
          any(a["atype"] == "OBSTACLE" for a in alerts),
          next((a["message"] for a in alerts if a["atype"] == "OBSTACLE"), "none"))

    # Hysteresis: just over the threshold must NOT count as clear.
    r = s.post(f"{base}/api/rover/ultrasonic",
               json={"distance": round(threshold + margin / 2, 3),
                     "sensor": "front"}, timeout=5).json()
    check("a reading inside the margin stays red",
          r["action"] == "stop", f"{threshold + margin / 2:.3f} m -> {r['action']}")

    # A recheck only counts once the wait has actually elapsed.
    before = r["attempts"]
    time.sleep(recheck + 0.2)
    r = s.post(f"{base}/api/rover/ultrasonic",
               json={"distance": round(threshold - 0.1, 2), "sensor": "front"},
               timeout=5).json()
    check("each recheck is counted", r["attempts"] > before,
          f"attempt {before} -> {r['attempts']}")

    r = s.post(f"{base}/api/rover/ultrasonic",
               json={"distance": 1.8, "sensor": "front"}, timeout=5).json()
    check("properly clear -> GREEN and GO",
          r["action"] == "go" and r["strip_state"] == "clear", r["message"])

    st = s.get(f"{base}/api/state", timeout=10).json()
    check("obstacle alert closes with it",
          not any(a["atype"] == "OBSTACLE" for a in st["alerts"]))
    check("no obstacles left open", not st["obstacles"],
          f"{len(st['obstacles'])} still open")

    # ---- 5b. the explicit obstacle endpoint still works ------------------
    r = s.post(f"{base}/api/rover/obstacle", json={
        "distance": 0.2, "sensor": "bumper", "cleared": False}, timeout=5)
    check("bumper-style obstacle report accepted",
          r.status_code == 200 and r.json()["blocked"], r.text[:70])
    s.post(f"{base}/api/rover/obstacle",
           json={"sensor": "bumper", "cleared": True}, timeout=5)

    # ---- 6. inspection with an explicit bay ------------------------------
    target = next(sh for sh in shelves if sh["shelf_id"] == first["shelf"])
    product = target["items"][0]["product"]
    r = s.post(f"{base}/api/inspection", json={
        "shelf_id": target["shelf_id"],
        "detections": [{"product": product, "qty": 9, "confidence": 0.93}]}, timeout=5)
    check("inspection — explicit bay", r.status_code == 200, r.text[:70])

    # ---- 7. inspection with NO bay: inferred from pose -------------------
    r = s.post(f"{base}/api/inspection", json={
        "detections": [{"product": product, "qty": 8, "confidence": 0.9}],
        "pose": {"x": target["scan_x"], "y": target["scan_y"],
                 "theta": 1.5708 if target["y"] > target["scan_y"] else -1.5708}},
        timeout=5)
    ok = r.status_code == 200 and r.json().get("shelf_id") == target["shelf_id"]
    check("inspection — bay inferred from pose", ok,
          r.json().get("shelf_id", "") if r.status_code == 200 else r.text[:60])

    # ---- 8. misplacement -------------------------------------------------
    other = next(sh for sh in shelves
                 if sh["shelf_id"] != target["shelf_id"] and sh["items"])
    stray = other["items"][0]["product"]
    s.post(f"{base}/api/inspection", json={
        "shelf_id": target["shelf_id"],
        "detections": [{"product": product, "qty": 9, "confidence": 0.9},
                       {"product": stray, "qty": 1, "confidence": 0.8}]}, timeout=5)
    alerts = s.get(f"{base}/api/state", timeout=10).json()["alerts"]
    hit = [a for a in alerts if a["atype"] == "MISPLACED" and a["product"] == stray]
    check("misplacement raises an alert", bool(hit),
          hit[0]["message"] if hit else "no alert raised")
    check("alert names the correct home bay",
          bool(hit) and other["shelf_id"] in hit[0]["message"],
          hit[0]["message"] if hit else "")

    # ---- 9. it clears on the next clean scan -----------------------------
    s.post(f"{base}/api/inspection", json={
        "shelf_id": target["shelf_id"],
        "detections": [{"product": product, "qty": 9, "confidence": 0.9}]}, timeout=5)
    alerts = s.get(f"{base}/api/state", timeout=10).json()["alerts"]
    check("alert auto-resolves on the next clean scan",
          not [a for a in alerts if a["atype"] == "MISPLACED" and a["product"] == stray])

    # ---- 10. low stock ---------------------------------------------------
    expected = target["items"][0]["expected"]
    s.post(f"{base}/api/inspection", json={
        "shelf_id": target["shelf_id"],
        "detections": [{"product": product, "qty": 1, "confidence": 0.9}]}, timeout=5)
    st = s.get(f"{base}/api/state", timeout=10).json()
    check("low stock raises an alert",
          any(a["atype"] == "LOW_STOCK" and a["product"] == product
              for a in st["alerts"]), f"1 of {expected} units")

    # ---- 11. forecasting --------------------------------------------------
    fc = st["forecasts"]
    check("forecast produced", len(fc) > 0, f"{len(fc)} products")
    check("forecast gives a real consumption rate",
          any(f["consumption_per_day"] > 0 for f in fc),
          f"max {max((f['consumption_per_day'] for f in fc), default=0)}/day")

    # ---- 12. timestamps ---------------------------------------------------
    ts = st["alerts"][0]["ts"] if st["alerts"] else None
    check("timestamps carry a UTC offset",
          bool(ts) and ("+" in ts[10:] or ts.endswith("Z")), ts or "no alerts")

    # ---- 13. bad input rejected -------------------------------------------
    r = s.post(f"{base}/api/inspection",
               json={"shelf_id": "ZZ9", "detections": []}, timeout=5)
    check("unknown bay rejected", r.status_code == 400, f"HTTP {r.status_code}")

    failed = [n for ok_, n in results if not ok_]
    print(f"\n{len(results) - len(failed)}/{len(results)} checks passed")
    if failed:
        print("failed: " + ", ".join(failed))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
