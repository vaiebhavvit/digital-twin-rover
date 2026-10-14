"""Backfill inspection history.

The forecast panel needs a trend before it can show one. This writes a week of
plausible inspections so the dashboard is populated the moment you open it —
useful for the review, and for developing the frontend without waiting for a
rover to drive six laps.

    python -m tools.seed_history            # 7 days, 5 laps a day
    python -m tools.seed_history --days 14
"""

from __future__ import annotations

import argparse
import random
import sys
from datetime import timedelta
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import yaml

from twin.config import settings
from twin.db import Inspection, repo, utcnow
from twin.engine import classify
from twin.models import LayoutDef


def seed(days: int = 7, laps_per_day: int = 5, seed_value: int = 7):
    layout = LayoutDef(**yaml.safe_load(settings.layout_file.read_text(encoding="utf-8")))
    repo.load_layout(layout)

    with repo.Session() as s:
        if s.query(Inspection).count() > 0:
            print("  history already present — skipping seed")
            return

    rng = random.Random(seed_value)
    start = utcnow() - timedelta(days=days)
    total_laps = days * laps_per_day
    hours_between = (days * 24) / total_laps

    rows = []
    for shelf in layout.shelves:
        for product, expected in shelf.products.items():
            level = float(expected)
            drain = rng.uniform(0.4, 1.9)          # units consumed per day
            for lap in range(total_laps):
                ts = start + timedelta(hours=lap * hours_between)
                level -= drain / laps_per_day
                level += rng.uniform(-0.25, 0.25)  # detection noise
                if level < 0.5 and rng.random() < 0.3:
                    level = float(expected)        # someone restocked
                level = max(0.0, level)
                qty = int(round(level))
                rows.append(Inspection(
                    ts=ts, shelf_id=shelf.shelf_id, product=product,
                    detected_qty=qty, confidence=round(rng.uniform(0.87, 0.98), 2),
                    status=classify(qty, expected)))

    with repo.Session() as s:
        s.add_all(rows)
        s.commit()
    print(f"  seeded {len(rows)} inspections across {days} days")


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


if __name__ == "__main__":
    _utf8_console()
    ap = argparse.ArgumentParser()
    ap.add_argument("--days", type=int, default=7)
    ap.add_argument("--laps-per-day", type=int, default=5)
    args = ap.parse_args()
    seed(args.days, args.laps_per_day)
