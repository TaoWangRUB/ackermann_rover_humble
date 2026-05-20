"""CC-5: Dashboard Server — FastAPI REST + WebSocket endpoints."""
import asyncio
import json
import logging
from typing import Any

from cc.event_bus import EventBus
from cc.telemetry_cache import TelemetryCache

logger = logging.getLogger(__name__)

try:
    from fastapi import FastAPI, WebSocket, WebSocketDisconnect
    from fastapi.staticfiles import StaticFiles
    from fastapi.responses import FileResponse
    import uvicorn
    HAS_FASTAPI = True
except ImportError:
    HAS_FASTAPI = False
    logger.warning("fastapi/uvicorn not installed — dashboard disabled")

# Protobuf
try:
    from google.protobuf.json_format import MessageToDict
except ImportError:
    MessageToDict = None


class DashboardServer:
    """FastAPI-based dashboard with REST and WebSocket endpoints."""

    def __init__(self, bus: EventBus, cache: TelemetryCache,
                 gateway: Any, ack_tracker: Any,
                 cfg: dict[str, Any]) -> None:
        self._bus = bus
        self._cache = cache
        self._gateway = gateway
        self._ack_tracker = ack_tracker
        self._host = cfg.get("host", "0.0.0.0")
        self._port = cfg.get("port", 8080)
        self._ws_rate_hz = cfg.get("ws_health_rate_hz", 2.0)

        self._app: Any = None
        self._server: Any = None
        self._health_ws_clients: set[Any] = set()
        self._alert_ws_clients: set[Any] = set()
        self._cmd_ack_ws_clients: set[Any] = set()
        self._hw_mode: dict[str, str] = {}

        if HAS_FASTAPI:
            self._build_app()

        self._px4_hw: dict = {}  # latest PX4 HW metrics from MAVLink

        # Subscribe to events for WS broadcast
        bus.subscribe("evt.health", self._broadcast_health)
        bus.subscribe("evt.alert", self._broadcast_alert)
        bus.subscribe("evt.cmd_ack_resolved", self._broadcast_cmd_ack)
        bus.subscribe("evt.px4_hw", self._on_px4_hw)

    def _build_app(self) -> None:
        self._app = FastAPI(title="Rover Control Center")

        @self._app.get("/api/status")
        async def get_status() -> dict:
            all_data = await self._cache.get_all()
            result = {}
            for key, (data, stale) in all_data.items():
                if data is not None and MessageToDict:
                    result[key] = {"data": MessageToDict(data), "stale": stale}
                else:
                    result[key] = {"data": None, "stale": stale}
            if MessageToDict:
                effective_health, effective_stale = await self._cache.get_effective_health()
                result["health"] = {
                    "data": MessageToDict(effective_health) if effective_health is not None else None,
                    "stale": effective_stale,
                }
            return result

        @self._app.post("/api/command")
        async def post_command(body: dict) -> dict:
            cmd_type = body.get("cmd_type", "")
            params = body.get("params", {})
            issued_by = body.get("issued_by", "dashboard")
            try:
                cmd_id = await self._gateway.send_command(
                    cmd_type, params, issued_by)
                return {"cmd_id": cmd_id, "status": "sent"}
            except ValueError as e:
                await self._ack_tracker.record_local_rejection(cmd_type, str(e))
                return {"error": str(e), "status": "rejected"}

        @self._app.get("/api/command/history")
        async def get_command_history() -> list:
            return self._ack_tracker.get_history()

        @self._app.get("/api/hw_mode")
        async def get_hw_mode() -> dict:
            return self._hw_mode

        @self._app.post("/api/hw_mode")
        async def set_hw_mode(body: dict) -> dict:
            self._hw_mode = {
                k: v for k, v in body.items()
                if isinstance(k, str) and v in ("hw", "mock")
            }
            logger.info("HW mode updated: %s", self._hw_mode)
            return self._hw_mode

        @self._app.get("/api/health/history")
        async def get_health_history() -> dict:
            # Return current cache snapshot (InfluxDB has historical data)
            all_data = await self._cache.get_all()
            result = {}
            for key, (data, stale) in all_data.items():
                if data is not None and MessageToDict:
                    result[key] = MessageToDict(data)
            effective_health, _ = await self._cache.get_effective_health()
            if effective_health is not None and MessageToDict:
                result["health"] = MessageToDict(effective_health)
            return result

        # Serve React SPA static files
        import os
        static_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "static")
        if os.path.isdir(static_dir):
            # Serve index.html for SPA root
            @self._app.get("/")
            async def serve_root():
                return FileResponse(os.path.join(static_dir, "index.html"))

            # Serve static assets (JS, CSS, etc.)
            self._app.mount("/assets", StaticFiles(directory=os.path.join(static_dir, "assets")), name="assets")

        @self._app.websocket("/ws/health")
        async def ws_health(ws: WebSocket) -> None:
            await ws.accept()
            self._health_ws_clients.add(ws)
            try:
                if MessageToDict:
                    effective_health, _ = await self._cache.get_effective_health()
                    if effective_health is not None:
                        await ws.send_text(json.dumps(MessageToDict(effective_health)))
                while True:
                    await ws.receive_text()
            except WebSocketDisconnect:
                self._health_ws_clients.discard(ws)

        @self._app.websocket("/ws/alerts")
        async def ws_alerts(ws: WebSocket) -> None:
            await ws.accept()
            self._alert_ws_clients.add(ws)
            try:
                while True:
                    await ws.receive_text()
            except WebSocketDisconnect:
                self._alert_ws_clients.discard(ws)

        @self._app.websocket("/ws/cmd_ack")
        async def ws_cmd_ack(ws: WebSocket) -> None:
            await ws.accept()
            self._cmd_ack_ws_clients.add(ws)
            try:
                while True:
                    await ws.receive_text()
            except WebSocketDisconnect:
                self._cmd_ack_ws_clients.discard(ws)

    async def start(self) -> None:
        if not HAS_FASTAPI or not self._app:
            logger.warning("Dashboard server disabled (missing FastAPI)")
            return
        config = uvicorn.Config(
            self._app, host=self._host, port=self._port, log_level="info")
        self._server = uvicorn.Server(config)
        asyncio.create_task(self._server.serve())
        logger.info("Dashboard server started on %s:%d", self._host, self._port)

    async def stop(self) -> None:
        if self._server:
            self._server.should_exit = True
            logger.info("Dashboard server stopped")

    async def _on_px4_hw(self, data: Any = None, **kwargs: Any) -> None:
        if isinstance(data, dict):
            self._px4_hw = data

    async def _broadcast_health(self, data: Any = None, **kwargs: Any) -> None:
        if not self._health_ws_clients or not MessageToDict:
            return
        effective_health, _ = await self._cache.get_effective_health()
        health_dict = MessageToDict(effective_health if effective_health is not None else data)
        # Merge PX4 HW metrics (CPU/RAM from MAVLink) into the px4 sub-dict
        if self._px4_hw and "px4" in health_dict:
            for k in ("cpuLoadPct", "ramUsagePct"):
                snake = "".join(f"_{c.lower()}" if c.isupper() else c for c in k)
                if snake in self._px4_hw:
                    health_dict["px4"][k] = self._px4_hw[snake]
            # Also pass raw keys
            if "cpu_load_pct" in self._px4_hw:
                health_dict["px4"]["cpuLoadPct"] = self._px4_hw["cpu_load_pct"]
            if "ram_usage_pct" in self._px4_hw:
                health_dict["px4"]["ramUsagePct"] = self._px4_hw["ram_usage_pct"]
        payload = json.dumps(health_dict)
        dead: set[Any] = set()
        for ws in self._health_ws_clients:
            try:
                await ws.send_text(payload)
            except Exception:
                dead.add(ws)
        self._health_ws_clients -= dead

    async def _broadcast_alert(self, data: Any = None, **kwargs: Any) -> None:
        if not self._alert_ws_clients or not MessageToDict:
            return
        payload = json.dumps(MessageToDict(data))
        dead: set[Any] = set()
        for ws in self._alert_ws_clients:
            try:
                await ws.send_text(payload)
            except Exception:
                dead.add(ws)
        self._alert_ws_clients -= dead

    async def _broadcast_cmd_ack(self, data: Any = None, **kwargs: Any) -> None:
        if not self._cmd_ack_ws_clients:
            return
        payload = json.dumps(data if isinstance(data, dict) else kwargs)
        dead: set[Any] = set()
        for ws in self._cmd_ack_ws_clients:
            try:
                await ws.send_text(payload)
            except Exception:
                dead.add(ws)
        self._cmd_ack_ws_clients -= dead
