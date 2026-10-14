"""WebSocket fan-out.

Ingestion endpoints run in FastAPI's threadpool (they do blocking DB work), so
they publish through `publish_threadsafe`, which hands the coroutine back to
the event loop. Browsers get pushed updates instead of polling; a REST
snapshot endpoint remains available as a fallback.
"""

from __future__ import annotations

import asyncio
import json
import time

from fastapi import WebSocket


class Hub:
    def __init__(self):
        self._clients: set[WebSocket] = set()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._lock = asyncio.Lock()

    def bind_loop(self, loop: asyncio.AbstractEventLoop):
        self._loop = loop

    async def connect(self, ws: WebSocket):
        await ws.accept()
        async with self._lock:
            self._clients.add(ws)

    async def disconnect(self, ws: WebSocket):
        async with self._lock:
            self._clients.discard(ws)

    @property
    def client_count(self) -> int:
        return len(self._clients)

    async def broadcast(self, kind: str, payload: dict):
        if not self._clients:
            return
        frame = json.dumps({"type": kind, "payload": payload, "ts": time.time()})
        dead = []
        for ws in list(self._clients):
            try:
                await ws.send_text(frame)
            except Exception:
                dead.append(ws)
        if dead:
            async with self._lock:
                for ws in dead:
                    self._clients.discard(ws)

    def publish_threadsafe(self, kind: str, payload: dict):
        """Safe to call from a sync route handler running in the threadpool."""
        if self._loop is None or not self._clients:
            return
        try:
            asyncio.run_coroutine_threadsafe(self.broadcast(kind, payload), self._loop)
        except RuntimeError:
            pass


hub = Hub()
