"""Storage layer.

Two things live here:

  1. The ORM schema (SQLAlchemy 2.x) — portable across SQLite, PostgreSQL,
     MySQL. Change DT_DATABASE_URL, change nothing else.
  2. `TwinRepository`, the only interface the rest of the app talks to. If you
     ever need a store SQLAlchemy does not cover (MongoDB, Firebase, an
     existing WMS), subclass it, implement the same methods, and assign
     `twin.db.repo`. No other file changes.
"""

from __future__ import annotations

import json
from datetime import datetime, timedelta, timezone

from sqlalchemy import (Float, ForeignKey, Integer, String, Text, create_engine,
                        delete, func, select)
from sqlalchemy.orm import DeclarativeBase, Mapped, Session, mapped_column, sessionmaker

from .config import settings


def utcnow() -> datetime:
    return datetime.now(timezone.utc)


def iso_utc(dt: datetime | None) -> str | None:
    """Serialize a timestamp the browser cannot misread.

    SQLite drops tzinfo, so a round-tripped timestamp comes back naive. Sent
    without an offset, JavaScript's Date parses it as *local* time and every
    "x ago" label is wrong by the viewer's UTC offset. Everything is stored in
    UTC, so a naive value is tagged as UTC on the way out.
    """
    if dt is None:
        return None
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)
    return dt.isoformat()


class Base(DeclarativeBase):
    pass


# --------------------------------------------------------------------------
# Schema
# --------------------------------------------------------------------------
class Shelf(Base):
    __tablename__ = "shelves"
    shelf_id: Mapped[str] = mapped_column(String(16), primary_key=True)
    zone: Mapped[str] = mapped_column(String(8), default="A")
    x: Mapped[float] = mapped_column(Float)
    y: Mapped[float] = mapped_column(Float)
    w: Mapped[float] = mapped_column(Float)
    h: Mapped[float] = mapped_column(Float)
    scan_x: Mapped[float | None] = mapped_column(Float, nullable=True)
    scan_y: Mapped[float | None] = mapped_column(Float, nullable=True)


class Planogram(Base):
    """Expected state: which product belongs where, and how many."""
    __tablename__ = "planogram"
    shelf_id: Mapped[str] = mapped_column(ForeignKey("shelves.shelf_id"), primary_key=True)
    product: Mapped[str] = mapped_column(String(64), primary_key=True)
    expected_qty: Mapped[int] = mapped_column(Integer, default=0)


class Inspection(Base):
    """Observed state: one product, on one shelf, at one moment."""
    __tablename__ = "inspections"
    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    ts: Mapped[datetime] = mapped_column(default=utcnow, index=True)
    shelf_id: Mapped[str] = mapped_column(String(16), index=True)
    product: Mapped[str] = mapped_column(String(64), index=True)
    detected_qty: Mapped[int] = mapped_column(Integer, default=0)
    confidence: Mapped[float] = mapped_column(Float, default=0.0)
    status: Mapped[str] = mapped_column(String(16), default="OK")
    pose_x: Mapped[float | None] = mapped_column(Float, nullable=True)
    pose_y: Mapped[float | None] = mapped_column(Float, nullable=True)


class Alert(Base):
    __tablename__ = "alerts"
    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    ts: Mapped[datetime] = mapped_column(default=utcnow, index=True)
    severity: Mapped[str] = mapped_column(String(16), default="warning")
    atype: Mapped[str] = mapped_column(String(24), index=True)
    shelf_id: Mapped[str] = mapped_column(String(16), index=True)
    product: Mapped[str] = mapped_column(String(64), default="")
    message: Mapped[str] = mapped_column(Text)
    resolved: Mapped[int] = mapped_column(Integer, default=0, index=True)
    resolved_ts: Mapped[datetime | None] = mapped_column(nullable=True)


class TrajectoryPoint(Base):
    """Downsampled travelled path, kept for replay and the map overlay."""
    __tablename__ = "trajectory"
    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    ts: Mapped[datetime] = mapped_column(default=utcnow, index=True)
    x: Mapped[float] = mapped_column(Float)
    y: Mapped[float] = mapped_column(Float)
    theta: Mapped[float] = mapped_column(Float, default=0.0)
    nav_state: Mapped[str] = mapped_column(String(16), default="moving")


class KV(Base):
    """Small JSON blobs: rover state, active obstacles, layout metadata."""
    __tablename__ = "kv"
    key: Mapped[str] = mapped_column(String(64), primary_key=True)
    value: Mapped[str] = mapped_column(Text)
    ts: Mapped[datetime] = mapped_column(default=utcnow)


# --------------------------------------------------------------------------
# Repository
# --------------------------------------------------------------------------
class TwinRepository:
    """All persistence goes through this object."""

    def __init__(self, url: str | None = None):
        url = url or settings.database_url
        kwargs = {"future": True, "pool_pre_ping": True}
        if url.startswith("sqlite"):
            kwargs["connect_args"] = {"check_same_thread": False}
        self.engine = create_engine(url, **kwargs)
        self.Session = sessionmaker(self.engine, expire_on_commit=False)
        Base.metadata.create_all(self.engine)
        if url.startswith("sqlite"):
            with self.engine.begin() as c:
                from sqlalchemy import text
                c.execute(text("PRAGMA journal_mode=WAL"))

    # ---------------- layout ----------------
    def load_layout(self, layout):
        """Idempotent upsert of shelves + planogram from the YAML layout."""
        with self.Session() as s:
            for d in layout.shelves:
                row = s.get(Shelf, d.shelf_id) or Shelf(shelf_id=d.shelf_id)
                row.zone, row.x, row.y, row.w, row.h = d.zone, d.x, d.y, d.w, d.h
                row.scan_x = d.scan_x if d.scan_x is not None else d.x + d.w / 2
                row.scan_y = d.scan_y if d.scan_y is not None else d.y - 1.5
                s.merge(row)
                s.execute(delete(Planogram).where(Planogram.shelf_id == d.shelf_id))
                for product, qty in d.products.items():
                    s.add(Planogram(shelf_id=d.shelf_id, product=product,
                                    expected_qty=int(qty)))
            s.commit()

    def shelves(self) -> list[Shelf]:
        with self.Session() as s:
            return list(s.scalars(select(Shelf).order_by(Shelf.shelf_id)))

    def shelf_defs(self):
        """Shelves as ShelfDef models — what the route generator needs to
        place inspection waypoints and size each bay's strip."""
        from .models import ShelfDef
        plan = self.planogram()
        return [ShelfDef(shelf_id=s.shelf_id, zone=s.zone, x=s.x, y=s.y,
                         w=s.w, h=s.h, scan_x=s.scan_x, scan_y=s.scan_y,
                         products=plan.get(s.shelf_id, {}))
                for s in self.shelves()]

    def planogram(self) -> dict[str, dict[str, int]]:
        """shelf_id -> {product: expected_qty}"""
        out: dict[str, dict[str, int]] = {}
        with self.Session() as s:
            for p in s.scalars(select(Planogram)):
                out.setdefault(p.shelf_id, {})[p.product] = p.expected_qty
        return out

    def home_shelf(self, product: str) -> str | None:
        with self.Session() as s:
            return s.scalar(select(Planogram.shelf_id)
                            .where(Planogram.product == product).limit(1))

    # ---------------- inspections ----------------
    def add_inspections(self, rows: list[dict]):
        with self.Session() as s:
            s.add_all([Inspection(**r) for r in rows])
            s.commit()

    def latest_by_shelf(self) -> dict[tuple[str, str], Inspection]:
        """Most recent inspection row per (shelf, product)."""
        with self.Session() as s:
            newest = (select(Inspection.shelf_id, Inspection.product,
                             func.max(Inspection.id).label("mid"))
                      .group_by(Inspection.shelf_id, Inspection.product).subquery())
            q = select(Inspection).join(newest, Inspection.id == newest.c.mid)
            return {(i.shelf_id, i.product): i for i in s.scalars(q)}

    def product_history(self, product: str, limit: int = 60) -> list[tuple[datetime, int]]:
        with self.Session() as s:
            rows = s.execute(
                select(Inspection.ts, Inspection.detected_qty)
                .where(Inspection.product == product, Inspection.status != "MISPLACED")
                .order_by(Inspection.id.desc()).limit(limit)).all()
        return [(ts, qty) for ts, qty in reversed(rows)]

    def shelf_history(self, shelf_id: str, limit: int = 50):
        with self.Session() as s:
            return s.execute(
                select(Inspection.ts, Inspection.product, Inspection.detected_qty,
                       Inspection.status, Inspection.confidence)
                .where(Inspection.shelf_id == shelf_id)
                .order_by(Inspection.id.desc()).limit(limit)).all()

    def count_inspections(self) -> int:
        with self.Session() as s:
            return s.scalar(select(func.count(Inspection.id))) or 0

    # ---------------- alerts ----------------
    def open_alerts(self, limit: int = 50) -> list[Alert]:
        with self.Session() as s:
            return list(s.scalars(
                select(Alert).where(Alert.resolved == 0)
                .order_by(Alert.severity.desc(), Alert.id.desc()).limit(limit)))

    def raise_alert(self, severity, atype, shelf_id, product, message) -> bool:
        """Insert unless an identical alert is already open. True if created."""
        with self.Session() as s:
            dup = s.scalar(select(Alert.id).where(
                Alert.resolved == 0, Alert.atype == atype,
                Alert.shelf_id == shelf_id, Alert.product == (product or "")))
            if dup:
                return False
            s.add(Alert(severity=severity, atype=atype, shelf_id=shelf_id,
                        product=product or "", message=message))
            s.commit()
            return True

    def resolve_stale(self, shelf_id: str, keep: set[tuple[str, str]]) -> int:
        """Close alerts on this shelf the newest scan no longer supports."""
        n = 0
        with self.Session() as s:
            for a in s.scalars(select(Alert).where(Alert.resolved == 0,
                                                   Alert.shelf_id == shelf_id)):
                if (a.atype, a.product) not in keep:
                    a.resolved, a.resolved_ts = 1, utcnow()
                    n += 1
            s.commit()
        return n

    def resolve_alerts_of_type(self, atype: str, product: str | None = None) -> int:
        """Close every open alert of one kind — e.g. all obstacle stops for a
        sensor once the rover reports the way is clear."""
        n = 0
        with self.Session() as s:
            q = select(Alert).where(Alert.resolved == 0, Alert.atype == atype)
            if product is not None:
                q = q.where(Alert.product == product)
            for a in s.scalars(q):
                a.resolved, a.resolved_ts = 1, utcnow()
                n += 1
            s.commit()
        return n

    def resolve_alert(self, alert_id: int):
        with self.Session() as s:
            a = s.get(Alert, alert_id)
            if a:
                a.resolved, a.resolved_ts = 1, utcnow()
                s.commit()

    # ---------------- navigation ----------------
    def add_trajectory(self, x, y, theta, nav_state="moving"):
        with self.Session() as s:
            s.add(TrajectoryPoint(x=x, y=y, theta=theta, nav_state=nav_state))
            n = s.scalar(select(func.count(TrajectoryPoint.id))) or 0
            if n > settings.max_trajectory_points:
                cutoff = s.scalar(select(TrajectoryPoint.id)
                                  .order_by(TrajectoryPoint.id)
                                  .offset(n - settings.max_trajectory_points).limit(1))
                s.execute(delete(TrajectoryPoint).where(TrajectoryPoint.id < cutoff))
            s.commit()

    def trajectory(self, limit: int = 2000) -> list[list[float]]:
        with self.Session() as s:
            rows = s.execute(select(TrajectoryPoint.x, TrajectoryPoint.y)
                             .order_by(TrajectoryPoint.id.desc()).limit(limit)).all()
        return [[round(x, 3), round(y, 3)] for x, y in reversed(rows)]

    def clear_trajectory(self):
        with self.Session() as s:
            s.execute(delete(TrajectoryPoint))
            s.commit()

    # ---------------- key/value ----------------
    def put(self, key: str, value):
        with self.Session() as s:
            row = s.get(KV, key)
            if row:
                row.value, row.ts = json.dumps(value), utcnow()
            else:
                s.add(KV(key=key, value=json.dumps(value)))
            s.commit()

    def get(self, key: str, default=None):
        with self.Session() as s:
            row = s.get(KV, key)
        return json.loads(row.value) if row else default

    # ---------------- maintenance ----------------
    def purge_inspections_older_than(self, days: int):
        with self.Session() as s:
            s.execute(delete(Inspection)
                      .where(Inspection.ts < utcnow() - timedelta(days=days)))
            s.commit()


repo = TwinRepository()
