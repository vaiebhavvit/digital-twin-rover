/* Warehouse Digital Twin — dashboard client.
   State arrives over a WebSocket; the floor, the patrol route and the rover
   are drawn on a canvas in world coordinates (metres), so everything shares
   one frame with config/layout.yaml and config/route.yaml. */

"use strict";

const S = {
  shelves: [], alerts: [], forecasts: [], kpi: {}, rover: {},
  route: {waypoints: [], warnings: []}, obstacles: [], trajectory: [],
  strips: [], strip_state: {states: {}}, decision: null,
  labels: {}, layout: {},
};

const view = {cx: 10, cy: 6, scale: 40, follow: false, fitted: false};
const layers = {strips: true, labels: true, waypoints: false,
                bays: true, path: false};
let selected = null, selectedWp = null;

const $ = (id) => document.getElementById(id);

/* ====================================================================== */
/* Transport                                                              */
/* ====================================================================== */
let ws = null, wsRetry = 0, pollTimer = null;

function connect() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  ws = new WebSocket(`${proto}://${location.host}/ws`);

  ws.onopen = () => {
    wsRetry = 0;
    setLink("live", "Live", "websocket");
    if (pollTimer) { clearInterval(pollTimer); pollTimer = null; }
  };
  ws.onmessage = (ev) => {
    const {type, payload} = JSON.parse(ev.data);
    if (type === "snapshot") applySnapshot(payload);
    else if (type === "pose") { S.rover = {...S.rover, ...payload, online: true}; onPose(payload); }
    else if (type === "inspection") applyInspection(payload);
    else if (type === "alerts") { S.alerts = payload.alerts; renderAlerts(); }
    else if (type === "strips") applyStrips(payload);
    else if (type === "route") { S.route = payload; renderRoute(); }
    renderHeader();
  };
  ws.onclose = () => {
    setLink("bad", "Reconnecting", "live stream dropped");
    wsRetry += 1;
    startPolling();
    setTimeout(connect, Math.min(1000 * wsRetry, 8000));
  };
  ws.onerror = () => ws.close();
}

function startPolling() {
  if (pollTimer) return;
  pollTimer = setInterval(async () => {
    try {
      const r = await fetch("/api/state", {cache: "no-store"});
      applySnapshot(await r.json());
      setLink("warn", "Polling", "websocket unavailable");
    } catch (e) { setLink("bad", "Offline", "cannot reach the server"); }
  }, 2000);
}

function setLink(cls, label, detail) {
  $("linkdot").className = "dot " + cls;
  $("linkstate").textContent = label;
  $("linkdetail").textContent = detail;
}

/* ====================================================================== */
/* State                                                                  */
/* ====================================================================== */
function applySnapshot(p) {
  Object.assign(S, p);
  if (p.layout && p.layout.name) $("layoutname").textContent =
    `${p.layout.name} — live replica of the floor and the inspection rover`;
  if (!view.fitted) fitToContent();
  renderAll();
}

function applyInspection(p) {
  S.shelves = p.shelves;
  S.alerts = p.alerts;
  if (p.pose) S.rover = {...S.rover, ...p.pose, online: true};
  renderShelvesDependent();
  flashBay(p.shelf_id);
  if (ws && ws.readyState === 1) ws.send("refresh");   // refresh KPIs/forecast
}

function applyStrips(p) {
  S.obstacles = p.obstacles || [];
  if (p.strips) S.strip_state = p.strips;
  if (p.decision) S.decision = {...p.decision, at: Date.now()};
  if (p.alerts) S.alerts = p.alerts;
  if (p.pose) S.rover = {...S.rover, ...p.pose};
  if (p.reset) { S.trajectory = []; S.decision = null; }
  renderAlerts(); renderSignal();
}

function onPose(p) {
  const t = S.trajectory;
  const last = t[t.length - 1];
  if (!last || Math.hypot(p.x - last[0], p.y - last[1]) > 0.1) {
    t.push([p.x, p.y]);
    if (t.length > 4000) t.shift();
  }
  if (view.follow) { view.cx = p.x; view.cy = p.y; }
  renderRouteProgress();
  renderSignal();
}

/* ====================================================================== */
/* Canvas                                                                 */
/* ====================================================================== */
const canvas = $("map");
const ctx = canvas.getContext("2d");
let W = 0, H = 0, flashes = {};

function resize() {
  const r = canvas.parentElement.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  W = r.width; H = r.height;
  canvas.width = Math.round(W * dpr);
  canvas.height = Math.round(H * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}
new ResizeObserver(resize).observe(canvas.parentElement);
resize();

const sx = (x) => (x - view.cx) * view.scale + W / 2;
const sy = (y) => H / 2 - (y - view.cy) * view.scale;
const wx = (px) => (px - W / 2) / view.scale + view.cx;
const wy = (py) => view.cy - (py - H / 2) / view.scale;

function contentBounds() {
  if (S.layout && S.layout.bounds && S.layout.bounds.length === 4) {
    const [a, b, c, d] = S.layout.bounds;
    return [a - 1, b - 1, c + 1, d + 1];
  }
  if (S.shelves.length) {
    const xs = S.shelves.flatMap(s => [s.x, s.x + s.w]);
    const ys = S.shelves.flatMap(s => [s.y, s.y + s.h]);
    return [Math.min(...xs) - 2, Math.min(...ys) - 2,
            Math.max(...xs) + 2, Math.max(...ys) + 2];
  }
  return [-1, -1, 21, 13];
}

function fitToContent() {
  const [a, b, c, d] = contentBounds();
  view.cx = (a + c) / 2;
  view.cy = (b + d) / 2;
  view.scale = Math.min(W / (c - a || 1), H / (d - b || 1)) * 0.95;
  view.fitted = true;
}

function flashBay(id) { if (id) flashes[id] = performance.now(); }

const STATUS_COLOR = {
  OK: "#4FB286", LOW_STOCK: "#E0A458", OUT_OF_STOCK: "#D96459",
  MISPLACED: "#B375DA", UNSCANNED: "#3B4D5E",
};
const ROUTE_COLOR = "#6E8BA8";
const ROUTE_ACTIVE = "#57B8FF";

function draw() {
  ctx.clearRect(0, 0, W, H);
  drawFloor();
  if (layers.path) drawTravelled();
  if (layers.strips) drawStrips();
  if (layers.waypoints) drawWaypoints();
  if (layers.bays) drawBays();
  drawObstacles();
  drawRover();
  drawScale();
  requestAnimationFrame(draw);
}

/* ---- the building ---- */
function drawFloor() {
  const b = (S.layout && S.layout.bounds) || [0, 0, 20, 12];
  const [minx, miny, maxx, maxy] = b;

  ctx.fillStyle = "#172230";
  ctx.fillRect(sx(minx), sy(maxy), (maxx - minx) * view.scale, (maxy - miny) * view.scale);

  // 1 m floor grid, drawn only when it would not turn into mush
  if (view.scale > 14) {
    ctx.strokeStyle = "rgba(60,80,100,.28)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let x = Math.ceil(minx); x <= maxx; x++) {
      ctx.moveTo(sx(x), sy(maxy)); ctx.lineTo(sx(x), sy(miny));
    }
    for (let y = Math.ceil(miny); y <= maxy; y++) {
      ctx.moveTo(sx(minx), sy(y)); ctx.lineTo(sx(maxx), sy(y));
    }
    ctx.stroke();
  }

  ctx.strokeStyle = "#3E5468";
  ctx.lineWidth = 2;
  ctx.strokeRect(sx(minx), sy(maxy), (maxx - minx) * view.scale,
                 (maxy - miny) * view.scale);

  (S.layout.obstacles || []).forEach(([x, y, w, h]) => {
    ctx.fillStyle = "rgba(62,84,104,.5)";
    ctx.fillRect(sx(x), sy(y + h), w * view.scale, h * view.scale);
  });

  if (S.layout && S.layout.dock) {
    const [dx, dy] = S.layout.dock;
    ctx.strokeStyle = "#4A6075";
    ctx.lineWidth = 1.2;
    ctx.strokeRect(sx(dx) - 11, sy(dy) - 11, 22, 22);
    ctx.fillStyle = "#6D8198";
    ctx.font = "11px ui-sans-serif,system-ui,sans-serif";
    // below the marker: waypoint labels are drawn above-right, and the dock
    // almost always has a waypoint sitting on it
    ctx.fillText("dock", sx(dx) - 11, sy(dy) + 26);
  }
}

/* ---- the patrol route, drawn as the strips the rover signals on ---- */
const STRIP_COLOR = {
  idle: "#55687E", clear: "#3FCF8E", blocked: "#F2544B", checking: "#E0A458",
};

function stripState(id) {
  const st = (S.strip_state.states || {})[id];
  return st ? st.state : "idle";
}

function drawStrips() {
  const strips = S.strips || [];
  if (!strips.length) return;
  const live = S.strip_state || {};
  const t = performance.now() / 1000;

  strips.forEach(st => {
    const state = stripState(st.strip_id);
    const isHere = live.current === st.strip_id;
    const isAhead = live.ahead === st.strip_id;
    const col = STRIP_COLOR[state] || STRIP_COLOR.idle;

    // Width encodes what the strip is: a bay frontage is the thick band the
    // rover inspects from, transit is thinner.
    let width = st.kind === "shelf" ? 9 : 6;
    let alpha = state === "idle" ? (st.kind === "shelf" ? 0.5 : 0.34) : 0.95;
    if (isHere) { width += 3; alpha = Math.max(alpha, 0.75); }

    if (state === "blocked") {
      // A halt is the one thing that should catch the eye across the room.
      const glow = 0.35 + 0.25 * Math.sin(t * 4);
      strokeStrip(st, col, width + 12, glow * 0.35);
    } else if (state === "clear") {
      strokeStrip(st, col, width + 8, 0.16);
    } else if (isAhead) {
      strokeStrip(st, "#8FA6BC", width + 6, 0.1);
    }

    strokeStrip(st, col, width, alpha);

    // A dark seam between strips so they read as separate blocks, not a hose.
    strokeStrip(st, "#101922", 2, 0.85, true);
  });

  if (layers.labels && view.scale > 17) {
    ctx.font = "600 10px ui-sans-serif,system-ui,sans-serif";
    ctx.textAlign = "center";
    strips.forEach(st => {
      const state = stripState(st.strip_id);
      const p = st.label_at;
      const outward = labelOffset(st);
      ctx.fillStyle = state === "idle" ? "#7E90A2" : STRIP_COLOR[state];
      const text = st.kind === "shelf" && st.shelf
        ? `${st.strip_id}·${st.shelf}` : st.strip_id;
      ctx.fillText(text, sx(p[0]) + outward[0], sy(p[1]) + outward[1]);
    });
    ctx.textAlign = "start";
  }
}

/* Offset a strip label towards the middle of the oval so it never lands on
   the bay rectangles sitting outside the track. The direction is computed in
   world space, where +y is up, then flipped for the canvas, where +y is
   down — without that flip the labels push outward, straight into the bays. */
function labelOffset(st) {
  const b = (S.layout && S.layout.bounds) || [0, 0, 24, 13];
  const cx = (b[0] + b[2]) / 2, cy = (b[1] + b[3]) / 2;
  const p = st.label_at;
  const dx = cx - p[0], dy = cy - p[1];
  const len = Math.hypot(dx, dy) || 1;
  return [(dx / len) * 16, -(dy / len) * 16 + 4];
}

function strokeStrip(st, color, widthPx, alpha, seamOnly) {
  const pts = st.points;
  if (!pts || pts.length < 2) return;
  ctx.save();
  ctx.globalAlpha = alpha;
  ctx.strokeStyle = color;
  ctx.lineWidth = widthPx;
  ctx.lineCap = seamOnly ? "butt" : "round";
  ctx.lineJoin = "round";
  if (seamOnly) {
    // just the two end caps, drawn as short ticks across the strip
    [pts[0], pts[pts.length - 1]].forEach((p, i) => {
      const q = i === 0 ? pts[1] : pts[pts.length - 2];
      const ang = Math.atan2(sy(q[1]) - sy(p[1]), sx(q[0]) - sx(p[0]))
                  + Math.PI / 2;
      const half = (st.kind === "shelf" ? 9 : 6) / 2 + 1;
      ctx.beginPath();
      ctx.moveTo(sx(p[0]) + Math.cos(ang) * half, sy(p[1]) + Math.sin(ang) * half);
      ctx.lineTo(sx(p[0]) - Math.cos(ang) * half, sy(p[1]) - Math.sin(ang) * half);
      ctx.stroke();
    });
    ctx.restore();
    return;
  }
  ctx.beginPath();
  pts.forEach((p, i) => i ? ctx.lineTo(sx(p[0]), sy(p[1]))
                          : ctx.moveTo(sx(p[0]), sy(p[1])));
  ctx.stroke();
  ctx.restore();
}

function drawWaypoints() {
  const wps = S.route.waypoints || [];
  wps.forEach(w => {
    const isStop = w.action === "inspect";
    const isDock = w.action === "dock";
    const px = sx(w.x), py = sy(w.y);
    const r = isStop || isDock ? 6 : 3.5;
    ctx.beginPath();
    ctx.arc(px, py, r, 0, Math.PI * 2);
    ctx.fillStyle = "#16202B";
    ctx.fill();
    ctx.strokeStyle = selectedWp === w.id ? "#FFFFFF"
                    : isStop ? "#57B8FF" : "#6E8BA8";
    ctx.lineWidth = selectedWp === w.id ? 2.2 : 1.3;
    ctx.stroke();
    if (view.scale > 15 && (isStop || isDock)) {
      ctx.fillStyle = "#A9C3DA";
      ctx.font = "10px ui-sans-serif,system-ui,sans-serif";
      ctx.fillText(w.id, px + r + 3, py - r - 1);
    }
  });
}

function drawTravelled() {
  if (S.trajectory.length < 2) return;
  ctx.strokeStyle = "rgba(95,208,196,.5)";
  ctx.lineWidth = 1.6;
  ctx.lineJoin = ctx.lineCap = "round";
  ctx.beginPath();
  S.trajectory.forEach((p, i) => i ? ctx.lineTo(sx(p[0]), sy(p[1]))
                                   : ctx.moveTo(sx(p[0]), sy(p[1])));
  ctx.stroke();
}

function drawBays() {
  const now = performance.now();
  S.shelves.forEach(s => {
    const col = STATUS_COLOR[s.status] || STATUS_COLOR.UNSCANNED;
    const x = sx(s.x), y = sy(s.y + s.h);
    const w = s.w * view.scale, h = s.h * view.scale;

    let glow = 0;
    if (flashes[s.shelf_id]) {
      const age = now - flashes[s.shelf_id];
      if (age > 1100) delete flashes[s.shelf_id]; else glow = 1 - age / 1100;
    }

    ctx.fillStyle = hexA(col, s.status === "UNSCANNED" ? 0.32 : 0.24 + glow * 0.3);
    roundRect(x, y, w, h, Math.min(4, w / 6));
    ctx.fill();
    ctx.strokeStyle = col;
    ctx.lineWidth = (selected === s.shelf_id ? 2.4 : 1.2) + glow * 1.6;
    ctx.stroke();

    if (w > 46) {
      ctx.fillStyle = "#DCE5ED";
      ctx.font = "600 12px ui-sans-serif,system-ui,sans-serif";
      ctx.fillText(s.shelf_id, x + 6, y + 15);
      const item = s.items.find(i => i.status !== "MISPLACED") || s.items[0];
      if (item && h > 30) {
        ctx.fillStyle = "#8EA2B4";
        ctx.font = "11px ui-sans-serif,system-ui,sans-serif";
        const name = fitText(item.product, w - 12);
        ctx.fillText(name, x + 6, y + 29);
        if (h > 44 && item.detected !== null) {
          ctx.fillStyle = "#C3D0DC";
          ctx.fillText(`${item.detected} of ${item.expected}`, x + 6, y + 42);
        }
      }
    }
  });
}

function drawObstacles() {
  const t = performance.now() / 1000;
  S.obstacles.forEach(o => {
    const x = sx(o.x), y = sy(o.y);
    const pulse = 10 + Math.sin(t * 3) * 3;
    ctx.fillStyle = "rgba(217,100,89,.16)";
    ctx.beginPath(); ctx.arc(x, y, pulse + 8, 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = "#D96459";
    ctx.beginPath(); ctx.arc(x, y, 5.5, 0, Math.PI * 2); ctx.fill();
    // No text: the red strip under it already says what this is, and a word
    // here lands on top of the strip labels.
  });
}

function drawRover() {
  const r = S.rover;
  if (!r || r.x === undefined) return;
  const x = sx(r.x), y = sy(r.y);
  const online = r.online !== false;
  const col = !online ? "#57687A"
            : r.state === "blocked" ? "#D96459"
            : r.state === "inspecting" ? "#4FB286" : "#57B8FF";

  if (r.state === "inspecting") {
    const th = r.theta || 0, span = 0.5, len = 2.4 * view.scale;
    ctx.fillStyle = hexA(col, 0.17);
    ctx.beginPath();
    ctx.moveTo(x, y);
    ctx.arc(x, y, len, -th - span, -th + span);
    ctx.closePath();
    ctx.fill();
  }

  ctx.strokeStyle = col; ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.arc(x, y, 10, 0, Math.PI * 2); ctx.stroke();
  ctx.fillStyle = col;
  ctx.beginPath(); ctx.arc(x, y, 5, 0, Math.PI * 2); ctx.fill();

  const th = r.theta || 0;
  ctx.beginPath();
  ctx.moveTo(x, y);
  ctx.lineTo(x + Math.cos(th) * 16, y - Math.sin(th) * 16);
  ctx.lineWidth = 2;
  ctx.stroke();
}

function drawScale() {
  const targets = [0.5, 1, 2, 5, 10, 20];
  const metres = targets.find(t => t * view.scale > 70) || 20;
  $("scalebar").style.width = (metres * view.scale) + "px";
  $("scalebar").style.height = "6px";
  $("scalelabel").textContent = metres + " m";
}

/* Trim a label to the width it has, so a long product name cannot run out of
   its bay and over the next one. */
function fitText(text, maxWidth) {
  if (ctx.measureText(text).width <= maxWidth) return text;
  let t = text;
  while (t.length > 1 && ctx.measureText(t + "…").width > maxWidth) {
    t = t.slice(0, -1);
  }
  return t + "…";
}

function roundRect(x, y, w, h, r) {
  ctx.beginPath();
  ctx.moveTo(x + r, y);
  ctx.arcTo(x + w, y, x + w, y + h, r);
  ctx.arcTo(x + w, y + h, x, y + h, r);
  ctx.arcTo(x, y + h, x, y, r);
  ctx.arcTo(x, y, x + w, y, r);
  ctx.closePath();
}

function hexA(hex, a) {
  const n = parseInt(hex.slice(1), 16);
  return `rgba(${n >> 16 & 255},${n >> 8 & 255},${n & 255},${a})`;
}

/* ---- interaction ---- */
let dragging = false, dragStart = null, moved = 0;

canvas.addEventListener("pointerdown", (e) => {
  dragging = true; moved = 0;
  dragStart = {x: e.clientX, y: e.clientY, cx: view.cx, cy: view.cy};
  canvas.classList.add("dragging");
  canvas.setPointerCapture(e.pointerId);
});
canvas.addEventListener("pointermove", (e) => {
  if (!dragging) return;
  const dx = e.clientX - dragStart.x, dy = e.clientY - dragStart.y;
  moved = Math.max(moved, Math.hypot(dx, dy));
  view.cx = dragStart.cx - dx / view.scale;
  view.cy = dragStart.cy + dy / view.scale;
  if (moved > 3) { view.follow = false; $("btn-follow").classList.remove("on"); }
});
canvas.addEventListener("pointerup", (e) => {
  dragging = false;
  canvas.classList.remove("dragging");
  if (moved < 4) pick(e);
});
canvas.addEventListener("wheel", (e) => {
  e.preventDefault();
  const r = canvas.getBoundingClientRect();
  const mx = e.clientX - r.left, my = e.clientY - r.top;
  const bx = wx(mx), by = wy(my);
  view.scale = Math.max(6, Math.min(260, view.scale * (e.deltaY < 0 ? 1.12 : 0.89)));
  view.cx = bx - (mx - W / 2) / view.scale;
  view.cy = by + (my - H / 2) / view.scale;
}, {passive: false});

function pick(e) {
  const r = canvas.getBoundingClientRect();
  const px = e.clientX - r.left, py = e.clientY - r.top;
  const x = wx(px), y = wy(py);

  // waypoints win: they are small and sit on top
  const wp = (S.route.waypoints || []).find(w =>
    Math.hypot(sx(w.x) - px, sy(w.y) - py) < 11);
  if (wp) { selectedWp = wp.id; selected = wp.shelf || null; renderDetail(); return; }

  const hit = S.shelves.find(s => x >= s.x && x <= s.x + s.w && y >= s.y && y <= s.y + s.h);
  selected = hit ? hit.shelf_id : null;
  selectedWp = null;
  renderDetail();
}

$("btn-fit").onclick = () => {
  view.follow = false; $("btn-follow").classList.remove("on"); fitToContent();
};
$("btn-follow").onclick = (e) => {
  view.follow = !view.follow;
  e.target.classList.toggle("on", view.follow);
  if (view.follow && S.rover.x !== undefined) {
    view.cx = S.rover.x; view.cy = S.rover.y;
    view.scale = Math.max(view.scale, 55);
  }
};
document.querySelectorAll("#layers button").forEach(b => {
  b.onclick = () => {
    const k = b.dataset.layer;
    layers[k] = !layers[k];
    b.classList.toggle("on", layers[k]);
  };
});

/* ====================================================================== */
/* Panels                                                                 */
/* ====================================================================== */
function label(s) { return S.labels[s] || s; }

function timeAgo(iso) {
  const d = (Date.now() - new Date(iso).getTime()) / 1000;
  if (d < 60) return Math.max(0, Math.round(d)) + "s ago";
  if (d < 3600) return Math.round(d / 60) + "m ago";
  if (d < 86400) return Math.round(d / 3600) + "h ago";
  return Math.round(d / 86400) + "d ago";
}

function sparkline(vals) {
  if (!vals || vals.length < 2) return "";
  const w = 62, h = 16, max = Math.max(...vals, 1);
  const step = w / (vals.length - 1);
  const pts = vals.map((v, i) =>
    `${(i * step).toFixed(1)},${(h - (v / max) * (h - 2) - 1).toFixed(1)}`).join(" ");
  return `<svg class="spark" width="${w}" height="${h}" viewBox="0 0 ${w} ${h}">
    <polyline points="${pts}" fill="none" stroke="#7E90A2" stroke-width="1.2"
    stroke-linejoin="round" stroke-linecap="round"/></svg>`;
}

function renderHeader() {
  const r = S.rover || {}, k = S.kpi || {};
  const online = r.online !== false && r.x !== undefined;
  const blocked = (S.obstacles || []).length > 0;

  $("roverdot").className = "dot " + (!online ? "bad" : blocked ? "warn" : "live");
  $("roverstate").textContent = r.x === undefined ? "Waiting for rover"
    : !online ? "Rover offline"
    : blocked ? "Stopped — path blocked"
    : r.state === "inspecting" ? `Inspecting ${r.current_shelf || ""}`
    : r.state === "charging" ? "Charging at dock"
    : r.state === "docked" ? "Docked"
    : r.state === "stopped" ? "Stopped"
    : "Following route";
  $("roverpos").textContent = r.x === undefined ? ""
    : `${r.x.toFixed(2)}, ${r.y.toFixed(2)} m   lap ${r.lap || 0}`;

  const b = Math.max(0, Math.round(r.battery ?? 0));
  const bf = $("battfill");
  bf.style.width = b + "%";
  bf.style.background = b < 20 ? "var(--out)" : b < 45 ? "var(--low)" : "var(--ok)";

  const prog = (r.route || {});
  $("routedot").className = "dot " + (!online ? "" : blocked ? "bad" : "live");
  $("routestate").textContent = S.route.name || "Route idle";
  $("routedetail").textContent = prog.progress_pct !== undefined
    ? `${prog.progress_pct}% of the lap · next ${prog.next_stop || "—"}`
    : `${(S.route.waypoints || []).length} waypoints`;

  const banner = $("banner");
  if (blocked) {
    const o = S.obstacles[0];
    const st = ((S.strip_state || {}).states || {})[o.strip] || {};
    banner.className = "banner show";
    banner.textContent = `Rover halted — ${o.sensor} sensor sees an obstacle `
      + `${o.distance != null ? o.distance.toFixed(2) + " m" : ""} ahead on strip `
      + `${o.strip || "?"}${o.shelf ? " (" + o.shelf + ")" : ""}. `
      + `Rechecking every ${(S.route.recheck_interval || 3).toFixed(0)} s`
      + `${st.attempts ? " — check " + st.attempts : ""}; `
      + `it moves off as soon as the strip reads clear.`;
  } else { banner.className = "banner"; }

  $("k-compliance").textContent = (k.compliance_pct ?? 0) + "%";
  $("k-inspec").textContent = `${k.shelves_ok ?? 0} / ${k.shelves_total ?? 0}`;
  $("k-critical").textContent = k.critical_alerts ?? 0;
  $("k-restock").textContent = k.restock_now ?? 0;
  $("k-progress").textContent = Math.round(prog.progress_pct ?? 0) + "%";
  $("k-scans").textContent = (k.total_scans ?? 0).toLocaleString();
  $("maptime").textContent = online
    ? `${S.route.name || "route"} · lap ${r.lap || 0}`
    : "waiting for the rover";
}

function renderAlerts() {
  const box = $("alerts");
  $("alertcount").textContent = S.alerts.length ? S.alerts.length + " open" : "";
  if (!S.alerts.length) {
    const scanned = (S.kpi && S.kpi.total_scans) || 0;
    box.innerHTML = `<div class="empty">${scanned
      ? "No open alerts. Every scanned bay matches its planogram."
      : "No inspections yet. Start the rover and scans will appear here."}</div>`;
    return;
  }
  box.innerHTML = S.alerts.map(a => `
    <div class="alert ${a.severity} ${a.atype}">
      <div class="bar"></div>
      <div style="flex:1">
        <div class="m">${esc(a.message)}</div>
        <div class="t">${esc(a.shelf_id)} · ${timeAgo(a.ts)}</div>
      </div>
      <button data-alert="${a.id}" title="Mark as handled">dismiss</button>
    </div>`).join("");
  box.querySelectorAll("button[data-alert]").forEach(b => {
    b.onclick = () => fetch(`/api/alerts/${b.dataset.alert}/resolve`, {method: "POST"});
  });
}

function renderLedger() {
  const rows = [];
  S.shelves.forEach(s => s.items.forEach(i => {
    const pct = i.detected === null ? 0
      : Math.min(100, Math.round(100 * i.detected / Math.max(i.expected, 1)));
    const col = STATUS_COLOR[i.status];
    rows.push(`<tr class="clickable" data-shelf="${s.shelf_id}">
      <td>${s.shelf_id}</td><td>${esc(i.product)}</td>
      <td>${i.detected === null ? "&ndash;" : i.detected}</td>
      <td class="muted">${i.expected || "&ndash;"}</td>
      <td><div class="fill"><i style="width:${pct}%;background:${col}"></i></div></td>
      <td><span class="tag ${i.status}">${label(i.status)}</span></td></tr>`);
  }));
  const tb = $("ledger");
  tb.innerHTML = rows.join("");
  tb.querySelectorAll("tr[data-shelf]").forEach(tr => {
    tr.onclick = () => { selected = tr.dataset.shelf; selectedWp = null;
                         renderDetail(); centreOn(selected); };
  });
}

function centreOn(shelfId) {
  const s = S.shelves.find(x => x.shelf_id === shelfId);
  if (!s) return;
  view.follow = false;
  $("btn-follow").classList.remove("on");
  view.cx = s.x + s.w / 2;
  view.cy = s.y + s.h / 2;
}

function renderForecast() {
  $("forecast").innerHTML = S.forecasts.map(f => {
    const days = f.days_to_stockout === null ? "stable"
      : f.days_to_stockout + (f.days_to_stockout === 1 ? " day" : " days");
    return `<tr class="clickable" data-shelf="${f.shelf_id}">
      <td>${esc(f.product)}<div class="muted" style="font-size:12px">
        ${f.shelf_id} · ${f.current_qty} left</div></td>
      <td>${sparkline(f.trend)}</td>
      <td class="muted">${f.consumption_per_day}</td>
      <td>${days}</td>
      <td class="p-${f.priority.replace(/\s/g, "")}">${f.priority}</td></tr>`;
  }).join("");
  $("forecast").querySelectorAll("tr[data-shelf]").forEach(tr => {
    tr.onclick = () => { selected = tr.dataset.shelf; selectedWp = null;
                         renderDetail(); centreOn(selected); };
  });
}

function renderDetail() {
  const box = $("detail"), title = $("detailtitle");

  if (selectedWp) {
    const w = (S.route.waypoints || []).find(v => v.id === selectedWp);
    if (w) {
      title.innerHTML = `Waypoint ${esc(w.id)} <em>${w.action}</em>`;
      let html = `<div class="row"><span class="muted">Position</span>
        <span>${w.x.toFixed(2)}, ${w.y.toFixed(2)} m</span></div>`;
      if (w.shelf) html += `<div class="row"><span class="muted">Inspects</span>
        <span>bay ${esc(w.shelf)}</span></div>`;
      if (w.heading !== null && w.heading !== undefined)
        html += `<div class="row"><span class="muted">Faces</span>
          <span>${w.heading}°</span></div>`;
      html += `<div class="row"><span class="muted">Pause</span><span>${
        w.pause !== null && w.pause !== undefined ? w.pause + " s"
        : (w.action === "inspect" ? "2.5 s (default)" : "none")}</span></div>`;
      html += `<div class="row"><span class="muted">Leg speed</span><span>${
        w.speed || S.route.default_speed} m/s</span></div>`;
      box.innerHTML = html;
      return;
    }
  }

  const s = S.shelves.find(x => x.shelf_id === selected);
  if (!s) {
    title.innerHTML = 'Bay detail <em>click a bay on the map</em>';
    box.innerHTML = '<div class="muted">Select any bay to see expected versus detected '
      + 'stock, misplaced items and the time of its last scan.</div>';
    return;
  }
  title.innerHTML = `Bay ${s.shelf_id} <em>zone ${s.zone}</em>`;
  let html = `<div class="row"><span class="muted">Bay status</span>
    <span class="tag ${s.status}">${label(s.status)}</span></div>`;
  s.items.forEach(i => {
    const d = i.detected === null ? "not scanned" : `${i.detected} of ${i.expected}`;
    html += `<div class="row"><span>${esc(i.product)}</span><span>
      <span class="muted">${d}</span> &nbsp;
      <span class="tag ${i.status}">${label(i.status)}</span></span></div>`;
  });
  html += `<div class="row"><span class="muted">Position</span><span>
    ${s.x.toFixed(1)}, ${s.y.toFixed(1)} m</span></div>`;
  html += `<div class="row"><span class="muted">Last scan</span><span>
    ${s.last_scan ? timeAgo(s.last_scan) : "never"}</span></div>`;
  box.innerHTML = html;
}

function renderSignal() {
  const d = S.decision;
  const live = S.strip_state || {};
  const states = live.states || {};
  const lamp = $("lamp"), title = $("signaltitle"), sub = $("signalsub");
  const cd = $("countdown"), head = $("signalstrip");

  const aheadId = live.ahead;
  const aheadState = aheadId ? (states[aheadId] || {}).state || "idle" : null;
  const blocked = (live.blocked || []).length > 0;

  head.textContent = aheadId
    ? `strip ahead ${aheadId}${stripShelf(aheadId) ? " · " + stripShelf(aheadId) : ""}`
    : "waiting for the rover";

  if (!d && !aheadState) {
    lamp.className = "lamp"; lamp.textContent = "—";
    title.textContent = "No reading yet";
    sub.textContent = "The rover has not reported its ultrasonic sensor";
    cd.style.display = "none";
    renderStripList();
    return;
  }

  if (blocked || aheadState === "blocked") {
    const sid = (live.blocked || [])[0] || aheadId;
    const st = states[sid] || {};
    lamp.className = "lamp stop"; lamp.textContent = "STOP";
    title.textContent = `Blocked on ${sid}` +
      (stripShelf(sid) ? ` (${stripShelf(sid)})` : "");
    sub.textContent = st.distance != null
      ? `${st.distance.toFixed(2)} m ahead on the ${st.sensor || "front"} sensor`
        + ` · check ${st.attempts || 1}`
      : "Rover halted, waiting for the path to clear";
    cd.style.display = "block";
    const remain = st.recheck_in_s;
    $("cdvalue").textContent = remain != null ? remain.toFixed(1) + "s" : "—";
  } else if (aheadState === "clear" || (d && d.clear)) {
    const sid = aheadId || (d && d.strip);
    const st = states[sid] || {};
    lamp.className = "lamp go"; lamp.textContent = "GO";
    title.textContent = `${sid || "Path"} clear`;
    sub.textContent = st.distance != null
      ? `${st.distance.toFixed(2)} m to the nearest echo — rover moving`
      : "Path clear — rover moving";
    cd.style.display = "none";
  } else {
    lamp.className = "lamp"; lamp.textContent = "—";
    title.textContent = aheadId ? `${aheadId} not yet checked` : "Idle";
    sub.textContent = "Waiting for the next ultrasonic reading";
    cd.style.display = "none";
  }
  renderStripList();
}

function stripShelf(id) {
  const st = (S.strips || []).find(s => s.strip_id === id);
  return st ? st.shelf : null;
}

function renderStripList() {
  const live = S.strip_state || {};
  const states = live.states || {};
  const rows = (S.strips || []).map(st => {
    const state = (states[st.strip_id] || {}).state || "idle";
    const info = states[st.strip_id] || {};
    const here = live.current === st.strip_id;
    const ahead = live.ahead === st.strip_id;
    if (state === "idle" && !here && !ahead) return "";
    const tag = here ? "rover here" : ahead ? "ahead" : "";
    const dist = info.distance != null ? `${info.distance.toFixed(2)} m` : "";
    return `<div class="striprow ${state} ${here ? "here" : ""}">
      <span class="chip"></span>
      <span class="sid">${st.strip_id}</span>
      <span>${esc(st.shelf || st.kind)}</span>
      <span class="meta">${[tag, dist].filter(Boolean).join(" · ")}</span>
    </div>`;
  }).filter(Boolean).join("");
  $("striplist").innerHTML = rows ||
    `<div class="striprow"><span class="meta" style="margin:0">
      ${(S.strips || []).length} strips armed, none active
     </span></div>`;
}

function renderRoute() {
  const r = S.route || {};
  $("routename").textContent = `${(r.waypoints || []).length} waypoints`;
  $("r-len").textContent = r.total_length_m ? r.total_length_m + " m" : "—";
  $("r-stops").textContent = r.inspect_stops ?? "—";
  const shelfStrips = (S.strips || []).filter(x => x.kind === "shelf").length;
  $("routename").textContent =
    `${(S.strips || []).length} strips · ${shelfStrips} at bays`;

  const box = $("routewarn");
  if (r.warnings && r.warnings.length) {
    box.className = "routewarn show";
    box.innerHTML = "<b>Route warnings</b>" + r.warnings.map(w =>
      `<div>${esc(w)}</div>`).join("");
  } else { box.className = "routewarn"; box.innerHTML = ""; }
  renderRouteProgress();
}

function renderRouteProgress() {
  const p = (S.rover.route || {});
  $("progbar").style.width = (p.progress_pct || 0) + "%";
  $("r-next").textContent = p.next_stop
    ? p.next_stop + (p.next_shelf && p.next_shelf !== p.next_stop
                     ? ` (${p.next_shelf})` : "")
    : "—";
  $("r-eta").textContent = p.eta_next_s !== undefined
    ? `${Math.round(p.eta_next_s)} s · ${p.distance_to_next_m} m` : "—";
  $("r-leg").textContent = p.leg_from ? `${p.leg_from} → ${p.leg_to}` : "—";
  const off = p.off_route_m;
  const el = $("r-off");
  el.textContent = off === undefined ? "—" : (off * 100).toFixed(0) + " cm";
  el.style.color = off > 0.6 ? "var(--out)" : off > 0.25 ? "var(--low)" : "";
}

function renderShelvesDependent() { renderAlerts(); renderLedger(); renderDetail(); }

function renderAll() {
  renderHeader(); renderShelvesDependent(); renderForecast();
  renderRoute(); renderSignal();
}

function esc(s) {
  return String(s).replace(/[&<>"]/g, c =>
    ({"&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;"}[c]));
}

/* ====================================================================== */
setInterval(() => { if (ws && ws.readyState === 1) ws.send("ping"); }, 25000);
setInterval(() => { if (S.alerts.length) renderAlerts(); }, 15000);
// Tick the recheck countdown locally so it runs down smoothly instead of
// jumping whenever the rover happens to send its next reading.
setInterval(() => {
  const states = (S.strip_state || {}).states || {};
  let live = false;
  Object.values(states).forEach(st => {
    if (st.state === "blocked" && st.recheck_in_s != null) {
      st.recheck_in_s = Math.max(0, st.recheck_in_s - 0.2);
      live = true;
    }
  });
  if (live) renderSignal();
}, 200);

connect();
startPolling();
requestAnimationFrame(draw);
