#!/usr/bin/env python3
"""Start the Digital Twin server.

    python run.py                 # http://localhost:8000
    python run.py --port 9000
    python run.py --reload        # auto-restart while you edit
    python run.py --seed          # backfill demo history, then start
"""

import argparse
import sys

import uvicorn

from twin.config import settings


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
    ap.add_argument("--host", default=settings.host)
    ap.add_argument("--port", type=int, default=settings.port)
    ap.add_argument("--reload", action="store_true")
    ap.add_argument("--seed", action="store_true",
                    help="backfill inspection history so the forecast panel "
                         "is populated immediately")
    ap.add_argument("--reset", action="store_true",
                    help="wipe the database and start clean")
    args = ap.parse_args()

    if args.reset:
        from twin.config import BASE_DIR
        db = BASE_DIR / "warehouse_twin.db"
        for p in (db, db.with_suffix(".db-wal"), db.with_suffix(".db-shm")):
            p.unlink(missing_ok=True)
        for p in settings.map_dir.glob("*.png"):
            p.unlink(missing_ok=True)
        print("database and map snapshots cleared")

    if args.seed:
        from tools.seed_history import seed
        seed()

    print("\n  Warehouse Digital Twin")
    print(f"  dashboard → http://localhost:{args.port}")
    print(f"  API docs  → http://localhost:{args.port}/docs\n")

    uvicorn.run("twin.app:app", host=args.host, port=args.port,
                reload=args.reload, log_level="warning")


if __name__ == "__main__":
    sys.exit(main())
