"""Application assembly."""

from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager

import yaml
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from . import route as route_mod
from .api import router
from .config import BASE_DIR, settings
from .db import repo
from .hub import hub
from .navigation import nav
from .models import LayoutDef


def load_layout() -> LayoutDef:
    """Read config/layout.yaml and sync it into the database."""
    if not settings.layout_file.exists():
        raise FileNotFoundError(f"layout file not found: {settings.layout_file}")
    layout = LayoutDef(**yaml.safe_load(settings.layout_file.read_text(encoding="utf-8")))
    repo.load_layout(layout)
    repo.put("layout_meta", {"name": layout.name, "frame_id": layout.frame_id,
                             "bounds": layout.bounds, "dock": layout.dock})
    return layout


@asynccontextmanager
async def lifespan(app: FastAPI):
    hub.bind_loop(asyncio.get_running_loop())
    layout = load_layout()
    shelves = repo.shelf_defs()
    r = route_mod.load_route(shelves=shelves)
    nav.bind(r, shelves)
    warnings = r.validate(repo.shelves())
    print(f"  layout : {layout.name} — {len(layout.shelves)} bays")
    print(f"  route  : {r.name} — {len(r.waypoints)} waypoints, "
          f"{len(r.inspect_stops)} inspection stops, {r.total_length:.1f} m")
    print(f"  strips : {len(nav.strips.strips)} "
          f"({sum(1 for s in nav.strips.strips if s.kind == 'shelf')} at bay "
          f"frontages) — stop at {r.stop_distance:.2f} m, "
          f"recheck every {r.recheck_interval:.0f} s")
    print(f"  store  : {settings.database_url}")
    for w in warnings:
        print(f"  WARNING: {w}")
    yield


def create_app() -> FastAPI:
    app = FastAPI(title="Warehouse Digital Twin",
                  description="Digital twin for the autonomous warehouse "
                              "inspection rover on a fixed patrol route",
                  version="3.0", lifespan=lifespan)

    # The rover and the dashboard may sit on different hosts.
    app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"],
                       allow_headers=["*"])

    app.include_router(router)
    app.mount("/static", StaticFiles(directory=str(BASE_DIR / "static")), name="static")

    @app.get("/")
    def dashboard():
        # The dashboard is a static page; all state arrives over /ws and /api.
        return FileResponse(BASE_DIR / "templates" / "dashboard.html",
                            media_type="text/html",
                            headers={"Cache-Control": "no-cache"})

    return app


app = create_app()
