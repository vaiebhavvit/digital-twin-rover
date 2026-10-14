# Digital Twin Dashboard — Oval-Route Warehouse Inspection Rover

**Autonomous Warehouse Rover for Vision-Guided Inspection, Digital Twin Monitoring, and Predictive Inventory Intelligence**
Capstone Project-1 · Vaiebhav Vats, Pratyaksh Gupta, Nilesh Parija
Guide: Dr. Ganala Santoshi · B.Tech CSE (AI & Robotics), VIT Chennai

Python end to end. The rover follows a fixed **oval track** past two rows of
shelving. The path is cut into **strips**, one across the frontage of every
bay. The ultrasonic sensor decides each strip's colour:

> **obstacle → strip turns RED, rover stops** · wait 3 s, look again ·
> **clear → strip turns GREEN, rover moves forward**

---

## 1. Run it in two minutes

**Python 3.10 or newer** on the machine running the server. The Jetson is a
separate matter — see section 6; `tools/rover_client.py` needs only `requests`
and `pyserial` and runs on Python 3.7+, so an older JetPack image is fine.

```bash
python3 -m venv .venv
source .venv/bin/activate        # Windows (Git Bash): source .venv/Scripts/activate
python -m pip install --upgrade pip   # a fresh Python ships an old pip that
                                      # fails to resolve some packages
pip install -r requirements.txt

python -m tools.seed_history     # a week of history so forecasts show up
python run.py                    # → http://localhost:8000
```

Second terminal:

```bash
python -m tools.rover_sim --speed 3
```

Watch a bay get blocked on demand:

```bash
python -m tools.rover_sim --speed 3 --drop B3 --drop-clears 15
```

A crate appears in the aisle before B3, the strip goes red, the rover halts,
the panel counts down 3 s per recheck, and when the crate is removed the strip
turns green and it drives on.

Useful flags: `python run.py --reset --seed` for a clean demo, `--reload`
while editing. `--obstacle-rate 0` for a calm run, `--drop-clears 0` to leave
an obstacle there for good.

---

## 2. Put it on GitHub for your team

Do this once, from the folder you just ran.

**Before the first commit**, check that the throwaway files are excluded. The
repo already has a `.gitignore` covering them, so this should print nothing
but the files you want:

```bash
git init -b main
git add -A
git status --short          # no warehouse_twin.db, no *.log, no __pycache__
```

The database is deliberately not tracked: it is generated from
`layout.yaml` plus whatever the rover has scanned, so committing it would mean
three people constantly overwriting each other's inspection history. Everyone
regenerates their own with `python -m tools.seed_history`.

```bash
git commit -m "Digital Twin dashboard: oval route, path strips, obstacle signalling"
```

Create the repo on GitHub (New repository → name it
`warehouse-digital-twin` → **Private** → do not add a README, you already have
one), then:

```bash
git remote add origin https://github.com/<your-username>/warehouse-digital-twin.git
git push -u origin main
```

Add your teammates: **Settings → Collaborators → Add people**. On a private
repo they cannot clone until they accept that invite.

### What your teammates run

```bash
git clone https://github.com/<your-username>/warehouse-digital-twin.git
cd warehouse-digital-twin
python -m venv .venv && source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r requirements.txt
python -m tools.seed_history
python run.py
```

Then `python -m tools.rover_sim --speed 3` in a second terminal. They have the
full dashboard, with no hardware.

### Working on it as three people

You own the dashboard, so keep `main` yours and have changes come in as
branches:

```bash
git checkout -b vision-integration     # whoever is working
# ...edit, test...
git add -A && git commit -m "Report detections from the trained YOLO model"
git push -u origin vision-integration
```

Then open a Pull Request on GitHub and merge it once it runs. Before starting
any new piece of work, `git pull` first — most merge pain on a three-person
project is someone branching off a week-old `main`.

**The files most likely to collide**, so agree who owns each:

| File | Owner | Why |
|---|---|---|
| `config/layout.yaml` | whoever measures the mock-up | physical measurements |
| `config/route.yaml` | whoever tunes the path | track shape and sensor thresholds |
| `firmware/rover_firmware.ino` | hardware | pins and calibration constants |
| `tools/rover_client.py` | hardware + vision | odometry and YOLO |
| `twin/`, `static/` | you | the twin itself |

## 3. The track and the strips

```
        ┌────┐ ┌────┐ ┌────┐ ┌────┐ ┌────┐ ┌────┐     zone B, y = 10.5 … 11.7
        └────┘ └────┘ └────┘ └────┘ └────┘ └────┘
   S17 ╭━━S16━━S15━━S14━━S13━━S12━━S11━━━━╮ S10      top straight, y = 10.0
       ┃                                  ┃
  dock ┃            oval track            ┃ S09       arcs, radius 3.5 m
       ┃                                  ┃
   S19 ╰━━S01━━S02━━S03━━S04━━S05━━S06━━━━╯ S07      bottom straight, y = 3.0
        ┌────┐ ┌────┐ ┌────┐ ┌────┐ ┌────┐ ┌────┐     zone A, y = 1.3 … 2.5
        └────┘ └────┘ └────┘ └────┘ └────┘ └────┘
```

The rover runs **counter-clockwise**, so the shelving is always on its right:
one camera on the right-hand side sees every bay, on both straights.

**20 strips**: 12 across bay frontages (`S01·A1` … `S16·B6`) and 8 transit
strips filling the arcs and the gaps. They tile the whole 49.9 m loop with no
gaps and no overlaps — the test suite checks that.

Each strip is one of:

| Colour | State | Meaning |
|---|---|---|
| grey | `idle` | not checked recently |
| **green** | `clear` | last reading was clear, rover driving |
| **red** | `blocked` | obstacle inside the stop distance, rover halted |

---

## 4. The obstacle cycle in detail

```
      rover drives, reading the sensor 4x a second
                        │
         ┌──────────────┴───────────────┐
    reading > 0.40 m              reading ≤ 0.40 m
         │                              │
    strip GREEN                   rover STOPS at once (firmware level)
    keep driving                  strip ahead turns RED
                                  alert raised on the dashboard
                                        │
                                  wait 3 seconds
                                        │
                                  read the sensor again
                                        │
                        ┌───────────────┴────────────────┐
                 still ≤ 0.50 m                    > 0.50 m
                 stays RED                         strip GREEN
                 attempt counter +1                rover moves forward
                 wait another 3 s                  alert closes itself
```

Three things in that diagram are deliberate and worth being able to defend:

**The stop happens three times over, independently.** The Arduino refuses to
drive forward under 30 cm; the Jetson halts the moment its own reading is
short, before it asks anything; the twin then confirms and colours the strip.
A stop that depends on a USB cable, a Python process and a Wi-Fi link is not a
stop. The firmware's threshold is deliberately *tighter* than the twin's, so
in normal running the Jetson stops first and the board is only a backstop.

**Clearing needs more room than stopping did** — 0.40 m to stop, 0.50 m to go
(`clear_margin: 0.10`). Without that gap an HC-SR04 reading 0.39 / 0.41 / 0.39
makes the rover lurch stop-go-stop-go. The margin turns a noisy boundary into
a decision.

**A red strip expires.** If the rover stops reporting — flat battery, crashed
process, Wi-Fi gone — the strip clears after 30 s rather than showing red
forever. A dashboard that lies about the floor is worse than one that admits
it does not know.

---

## 5. Changing the path

Everything about the track lives in **`config/route.yaml`**. The dashboard
draws it, the simulator drives it, and the real rover fetches it from
`GET /api/route`.

### Reshape the oval

```yaml
shape:
  type: oval
  center: [12.0, 6.5]        # middle of the track, metres
  straight_length: 14.0      # length of each straight
  radius: 3.5                # corner radius; the oval is 2 x this tall
  corner_samples: 12         # points per arc — more = smoother
  direction: ccw             # keeps the shelving on the rover's right
  dock_at: left
```

Change those six numbers and the track, the strips and the patrol all move
together. Bays attach themselves to the track by their `scan_x` / `scan_y`.

### Or write the path by hand

Delete the `shape:` block and list waypoints instead:

```yaml
waypoints:
  - { id: HOME, x: 1.5, y: 6.5, action: dock, heading: 180, pause: 4 }
  - { id: W1,   x: 5.0, y: 3.0 }
  - { id: A1,   x: 6.0, y: 3.0, action: inspect, shelf: A1, heading: -90 }
```

| Field | Meaning |
|---|---|
| `id` | short label drawn on the map |
| `x`, `y` | metres, same frame as `layout.yaml` |
| `action` | `move` · `inspect` · `dock` · `wait` |
| `shelf` | bay to inspect — required for `action: inspect` |
| `heading` | degrees faced on arrival; `0` = +x, `90` = +y |
| `pause` | seconds held here (inspect defaults to 2.5) |
| `speed` | m/s for the leg into this waypoint |

### Tune the obstacle behaviour

```yaml
obstacle_stop_distance: 0.40   # closer than this -> RED and stop
recheck_interval: 3.0          # seconds between rechecks
clear_margin: 0.10             # must exceed 0.50 m to count as clear

strips:
  shelf_margin: 0.20           # strip overhang each side of a bay
  max_transit_length: 3.0      # longest transit strip
  lookahead: 0.80              # how far ahead "the strip ahead" is
  clear_hold: 4.0              # how long green lingers after passing
  stale_after: 30.0            # drop a state the rover stopped reporting
```

Apply any edit without restarting:

```bash
curl -X POST http://localhost:8000/api/route/reload
```

### The route checks itself

On every load the twin validates the path against the floor plan and shows
the warnings on the dashboard: a waypoint inside a rack, a leg that cuts
through one, an `inspect` naming a bay that does not exist, a bay nothing ever
visits. You find out on screen, not by driving into a shelf.

---

## 6. Wiring the real ultrasonic sensor

Three layers, each able to stop the rover on its own.

### Layer 1 — the Arduino (`firmware/rover_firmware.ino`)

HC-SR04 on pins 12/13, L298N on 5/7/8 and 6/9/11, encoders on 2/3. Flash it
with the Arduino IDE, then **measure and set these three constants** — the
reported position depends entirely on them:

```cpp
const float WHEEL_DIAMETER_M = 0.065;   // measure your wheel
const int   TICKS_PER_REV    = 20;      // slots on your encoder disc
const float WHEEL_BASE_M     = 0.145;   // between the two wheels
```

Serial protocol, 115200 baud:

```
in    F<mm/s>  drive forward      out   P <dx> <dy> <dtheta>   odometry delta
      T<deg>   turn in place            U <cm>                 range, -1 = no echo
      S        stop                     B <volts>              battery
      ?        send telemetry now
```

Check it before connecting anything else:

```bash
screen /dev/ttyUSB0 115200      # you should see P / U / B lines at 10 Hz
```

Put your hand in front of the sensor — `U` should drop to the distance in cm.

### Layer 2 — the Jetson (`tools/rover_client.py`)

The twin server stays on the laptop; only this one file runs on the Jetson. It
imports nothing from `twin/`, so the Jetson never needs FastAPI, SQLAlchemy or
Python 3.10 — just:

```bash
pip3 install requests pyserial
pip3 install ultralytics opencv-python      # only when you add vision
```

Put both machines on the same Wi-Fi and find the laptop's address
(`ip addr` on Linux, `ipconfig` on Windows, `ipconfig getifaddr en0` on a Mac).
Check the Jetson can actually reach it **before** wiring anything up:

```bash
curl http://192.168.1.50:8000/api/health     # from the Jetson
```

No reply almost always means the laptop firewall is blocking port 8000, not
that the code is wrong — on Windows, allow Python through, and on Linux
`sudo ufw allow 8000`. The server already listens on `0.0.0.0`, so nothing in
the code needs changing. Then:

```bash
python3 tools/rover_client.py --server http://192.168.1.50:8000 \
                              --port /dev/ttyUSB0 --model best.pt
```

If the serial port is refused, add yourself to the dialout group once:
`sudo usermod -aG dialout $USER`, then log out and back in.

It pulls the route, drives it, and runs the obstacle cycle. Test navigation
before the vision model is ready with `--no-vision`.

### Layer 3 — the twin

```
POST /api/rover/ultrasonic
{"distance": 0.31, "sensor": "front",
 "pose": {"x": 6.0, "y": 3.0, "theta": -1.5708}}

→ {"clear": false, "action": "stop", "strip": "S01", "strip_state": "blocked",
   "recheck_in_s": 3.0, "attempts": 1,
   "message": "Obstacle 0.31 m ahead on S01 (A1) — stopped, rechecking in 3 s"}
```

Send a reading a few times a second. Act on `action`; when told to stop, send
another reading after `recheck_in_s`. A rover that decides everything for
itself can ignore the reply and just keep reporting — the dashboard stays
correct either way.

Other endpoints the rover uses:

```
POST /api/rover/pose        {"pose": {...}, "telemetry": {...}}   5–10 Hz
POST /api/inspection        {"shelf_id": "A1", "detections": [...]}
POST /api/rover/obstacle    for a bumper switch or boolean-only firmware
GET  /api/route             the waypoints to follow
GET  /api/strips            strip geometry and live state
```

`shelf_id` is optional on an inspection: omit it and send `pose`, and the twin
matches the scan to the nearest bay the camera is facing — so YOLO never has
to know the warehouse layout.

**The one coupling to get right:** your YOLO class names must match the
product strings in `config/layout.yaml` exactly.

---

## 7. Step by step, from here to a working demo

**Day 1 — the twin alone, no hardware.**
Run it, run the simulator, and get used to the dashboard. Everything you will
show the panel already works at this point. `python -m tools.test_integration`
proves the interface end to end — 38 checks.

**Day 2 — measure your mock-up.**
Build the shelf mock-up, then measure it with a tape and write the numbers
into `config/layout.yaml`: each bay's south-west corner, width, depth, and the
point on the track where the rover will sit to scan it. Set `bounds` to your
floor area. Reshape the oval in `config/route.yaml` so the straights run past
both rows with roughly 0.5 m clearance. Reload and read the warnings.

**Day 3 — the firmware.**
Flash `firmware/rover_firmware.ino`. Measure the wheel diameter, encoder slots
and wheel base, and set the three constants. Drive it with a serial monitor:
`F300` should move it, `S` should stop it, `U` should track your hand. Push the
rover one measured metre by hand and check the `P` lines add up to 1.0 — if
they do not, your `METRES_PER_TICK` is wrong and nothing downstream can fix it.

**Day 4 — the vision model.**
Photograph your 5–10 products on the shelves, 100+ images per class, label
them, train YOLO. **Name the classes exactly as in `layout.yaml`.** Test with
`--shelf A1` style single scans before running a full lap.

**Day 5 — closing the loop.**
Start the twin on the laptop, put the Jetson on the same Wi-Fi, run
`rover_client.py --no-vision` and watch the rover's dot track round the oval on
the dashboard. Fix the odometry constants until "off route by" stays under
20 cm for a full lap. Then turn vision on.

**Day 6 — rehearse.**
Run the demo script below three times. Record a screen capture of a clean lap
as a backup. Check the laptop's Wi-Fi is the one the Jetson expects, and know
the laptop's IP.

**On the day.** Start the twin first, then the rover. If the hardware misbehaves,
`python -m tools.rover_sim --speed 3` produces the same dashboard and nobody in
the room can tell. That is not a cheat — it is the reason the twin and the
rover talk over a defined contract instead of being one program.

---

## 8. Demo script for the review (about 5 minutes)

Before the panel arrives: `python run.py --reset --seed`, then
`python -m tools.rover_sim --speed 3`. Let one lap finish so the bays are green.

1. **Open on the map.** "This is the warehouse floor. The rover runs this oval
   track. The path is divided into strips — one across the front of every
   shelf — and each strip is a signal."
2. **Point at the green strip ahead of the rover.** "Green means the ultrasonic
   sensor has just checked that stretch and it is clear, so the rover is
   driving into it."
3. **Block it.** In another terminal:
   ```bash
   curl -X POST localhost:8000/api/rover/ultrasonic \
        -H 'content-type: application/json' \
        -d '{"distance":0.25,"sensor":"front"}'
   ```
   The strip snaps red, the lamp reads STOP, the banner names the strip and the
   bay, an alert opens, and a 3-second countdown starts.
4. **Let it recheck.** The countdown reaches zero, the attempt counter rises.
   "It will sit here as long as the aisle is blocked. It never guesses."
5. **Clear it.**
   ```bash
   curl -X POST localhost:8000/api/rover/ultrasonic \
        -H 'content-type: application/json' \
        -d '{"distance":1.5,"sensor":"front"}'
   ```
   Green, GO, the alert closes itself, the rover moves off.
6. **Let it reach a bay.** The camera cone appears, the bay flashes, the count
   updates. Show a misplacement alert if one has appeared: "Coffee Jar on A6.
   The planogram says B3. Nobody checked that by hand."
7. **Change the track live.** Open `config/route.yaml`, change `radius: 3.5` to
   `2.8`, save, and run `curl -X POST localhost:8000/api/route/reload`. The oval
   redraws and the strips rebuild. Put it back. "The path is configuration, not
   code."
8. **End on the restock forecast.** "Fitted to the rover's own inspection
   history, measuring depletion and ignoring refills."

Run `python -m tools.test_integration` in front of them if they ask how you
know the interface works.

---

## 9. Questions the panel will ask

**"Is this a digital twin or just a dashboard?"**
It holds expected state — the layout, the planogram, the route, the strips —
receives observed state from the rover, reconciles the two, and emits
decisions: stop or go, alerts, restocking priority, off-route warnings. The
`action` field the rover acts on comes *from the twin*. A dashboard only draws.

**"What if the Wi-Fi drops while an obstacle is in the way?"**
The rover still stops: the Arduino refuses to drive under 30 cm and the Jetson
halts on its own reading. `rover_client.py` falls back to its local rule and
keeps running the same 3-second recheck. The dashboard marks the rover offline
and expires the strip rather than showing a stale red.

**"Why strips rather than colouring the whole path?"**
A strip is a fixed, named stretch of floor: `S13` is always the same metre and
a half in front of bay B3. That makes an obstacle report something you can act
on — "blocked at B3" — instead of a moving dot. It is also what makes the
history meaningful: you can ask which strip blocks most often.

**"Why 3 seconds?"**
It is `recheck_interval` in `config/route.yaml` — one number, and the rover,
the simulator and the dashboard countdown all read it. Three seconds is long
enough for a person to step out of the aisle and short enough that the patrol
does not stall.

**"What stops the rover flickering between stop and go?"**
Hysteresis. It stops at 0.40 m but needs 0.50 m to call the path clear. Without
that an ultrasonic sitting on the threshold would oscillate.

**"How does it scale past 12 bays?"**
Layout and route are both data. Add bays to `layout.yaml`, and the strips
rebuild themselves around them — no code change. Storage is already
database-agnostic.

---

## 10. Project layout

```
twin/            the server
  config.py      every setting, overridable by environment variable
  models.py      the integration contract (Pydantic)
  db.py          ORM schema + TwinRepository — the only storage interface
  route.py       oval generator, geometry, progress, validation
  strips.py      strip map + the RED/GREEN state machine
  navigation.py  live pose, strips, obstacles
  engine.py      compliance rules, alert lifecycle, bay association
  forecast.py    consumption rate and restocking priority
  hub.py         WebSocket fan-out
  api.py         HTTP + WebSocket routes
  app.py         application assembly
config/
  layout.yaml    bays and planogram, in metres
  route.yaml     THE TRACK AND THE OBSTACLE RULES — edit this
firmware/
  rover_firmware.ino   Arduino: motors, HC-SR04, encoders, serial protocol
static/          dashboard CSS + canvas renderer
tools/
  rover_sim.py         simulated rover, drives the oval, full obstacle cycle
  rover_client.py      the real on-robot client for the Jetson
  seed_history.py      backfill inspection history
  test_integration.py  38 checks over the whole contract
```

---

## 11. Swapping the database

Everything goes through `TwinRepository`. For any SQL database, set one
variable:

```bash
export DT_DATABASE_URL="postgresql+psycopg://user:pass@host:5432/twin"
```

SQLite (default), PostgreSQL and MySQL all work unchanged; tables are created
on first run. For a store SQLAlchemy does not cover — MongoDB, Firebase, an
existing WMS — subclass `TwinRepository`, implement the same methods, and
assign `twin.db.repo`. No other file changes.

Settings, all environment variables: `DT_DATABASE_URL`, `DT_LAYOUT`,
`DT_ROUTE`, `DT_LOW_STOCK_RATIO`, `DT_SCAN_RADIUS`, `DT_ROVER_TIMEOUT`,
`DT_OFF_ROUTE`, `DT_TRAJ_STEP`, `DT_HOST`, `DT_PORT`, `DT_INGEST_TOKEN`.

Set `DT_INGEST_TOKEN` and every ingestion route requires an `X-Twin-Token`
header. Leave it empty on the lab network.

---

## 12. For Capstone-II

- AprilTag or line-following checkpoints at each waypoint to zero the odometry
  drift, which is the one thing that limits how long a lap can run.
- A side ultrasonic so the rover can tell a blocked aisle from a wide pallet it
  could pass.
- Logging which strips block most often — a heat map of where the warehouse
  actually gets congested, which is a genuinely new output.
- Multiple routes selectable at runtime, and a second rover (the pose API
  already carries enough to key on a rover id).
