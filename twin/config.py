"""Central configuration. Everything is overridable by environment variable,
so the same code runs on your laptop, on the Jetson, and on a server."""

import os
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent.parent


def _env(key, default):
    return os.getenv(key, default)


class Settings:
    # --- storage -----------------------------------------------------------
    # Any SQLAlchemy URL works. Switch database without touching code:
    #   sqlite:///warehouse_twin.db
    #   postgresql+psycopg://user:pass@host:5432/twin
    #   mysql+pymysql://user:pass@host:3306/twin
    database_url: str = _env("DT_DATABASE_URL",
                             f"sqlite:///{BASE_DIR / 'warehouse_twin.db'}")

    # Warehouse layout: shelves and planogram, in metres.
    layout_file: Path = Path(_env("DT_LAYOUT", str(BASE_DIR / "config" / "layout.yaml")))

    # The patrol route the rover follows. Edit this file to change the path.
    route_file: Path = Path(_env("DT_ROUTE", str(BASE_DIR / "config" / "route.yaml")))

    # --- twin rules --------------------------------------------------------
    low_stock_ratio: float = float(_env("DT_LOW_STOCK_RATIO", "0.4"))
    # A scan without an explicit shelf_id is matched to the nearest shelf
    # within this radius of the rover pose (metres).
    scan_association_radius: float = float(_env("DT_SCAN_RADIUS", "3.0"))
    # Rover considered offline after this many seconds of silence.
    rover_timeout_s: float = float(_env("DT_ROVER_TIMEOUT", "15"))

    # --- navigation --------------------------------------------------------
    # Persist a travelled-path point only after the rover has moved this far
    # (m). Keeps the DB small while pose still streams at full rate.
    trajectory_min_step: float = float(_env("DT_TRAJ_STEP", "0.15"))
    max_trajectory_points: int = int(_env("DT_TRAJ_MAX", "4000"))

    # --- server ------------------------------------------------------------
    host: str = _env("DT_HOST", "0.0.0.0")
    port: int = int(_env("DT_PORT", "8000"))
    # Shared secret for rover ingestion endpoints. Empty = open (lab default).
    ingest_token: str = _env("DT_INGEST_TOKEN", "")


    # How far the rover may drift from the route before the twin flags it (m).
    off_route_tolerance: float = float(_env("DT_OFF_ROUTE", "0.6"))


settings = Settings()
