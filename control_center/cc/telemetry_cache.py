"""CC-2: Telemetry Cache — thread-safe latest-value store with staleness detection."""
import asyncio
import copy
import logging
import time
from typing import Any

from cc.event_bus import EventBus

logger = logging.getLogger(__name__)


class TelemetryCache:
    """Asyncio-lock-protected dict keyed by subsystem with monotonic timestamps."""

    def __init__(self, bus: EventBus, cfg: dict[str, Any]) -> None:
        self._bus = bus
        self._stale_threshold = cfg.get("stale_threshold_s", 5.0)
        self._lock = asyncio.Lock()
        self._data: dict[str, Any] = {}
        self._updated_at: dict[str, float] = {}

        # Subscribe to health events
        bus.subscribe("evt.health", self._on_health)
        bus.subscribe("evt.health.cam", self._on_cam)
        bus.subscribe("evt.health.px4", self._on_px4)
        bus.subscribe("evt.health.jetson", self._on_jetson)

    async def _on_health(self, data: Any = None, **kwargs: Any) -> None:
        async with self._lock:
            self._data["health"] = data
            self._updated_at["health"] = time.monotonic()

    async def _on_cam(self, data: Any = None, **kwargs: Any) -> None:
        async with self._lock:
            self._data["cam"] = data
            self._updated_at["cam"] = time.monotonic()

    async def _on_px4(self, data: Any = None, **kwargs: Any) -> None:
        async with self._lock:
            self._data["px4"] = data
            self._updated_at["px4"] = time.monotonic()

    async def _on_jetson(self, data: Any = None, **kwargs: Any) -> None:
        async with self._lock:
            self._data["jetson"] = data
            self._updated_at["jetson"] = time.monotonic()

    async def get(self, key: str) -> tuple[Any | None, bool]:
        """Return (data, is_stale). Data is None if never received."""
        async with self._lock:
            data = self._data.get(key)
            updated = self._updated_at.get(key)
            if data is None or updated is None:
                return None, True
            age = time.monotonic() - updated
            return data, age > self._stale_threshold

    async def get_all(self) -> dict[str, tuple[Any, bool]]:
        """Return dict of {key: (data, is_stale)} for all cached items."""
        async with self._lock:
            now = time.monotonic()
            result = {}
            for key in self._data:
                data = self._data[key]
                updated = self._updated_at.get(key, 0)
                stale = (now - updated) > self._stale_threshold
                result[key] = (data, stale)
            return result

    async def get_effective_health(self) -> tuple[Any | None, bool]:
        """Return overall health with freshest PX4 data overlaid when available."""
        async with self._lock:
            health = self._data.get("health")
            health_updated = self._updated_at.get("health")
            px4 = self._data.get("px4")
            px4_updated = self._updated_at.get("px4")

            if health is None or health_updated is None:
                return None, True

            effective_health = copy.deepcopy(health)
            if px4 is not None and hasattr(effective_health, "px4"):
                effective_health.px4.CopyFrom(px4)

            latest_updated = health_updated
            if px4_updated is not None:
                latest_updated = max(latest_updated, px4_updated)

            stale = (time.monotonic() - latest_updated) > self._stale_threshold
            return effective_health, stale

    async def get_effective_px4(self) -> tuple[Any | None, bool]:
        """Return latest PX4 status, falling back to the overall health payload."""
        async with self._lock:
            px4 = self._data.get("px4")
            px4_updated = self._updated_at.get("px4")
            if px4 is not None and px4_updated is not None:
                stale = (time.monotonic() - px4_updated) > self._stale_threshold
                return px4, stale

            health = self._data.get("health")
            health_updated = self._updated_at.get("health")
            if health is None or health_updated is None:
                return None, True

            stale = (time.monotonic() - health_updated) > self._stale_threshold
            return getattr(health, "px4", None), stale

    def is_stale(self, key: str) -> bool:
        """Synchronous staleness check (for safety gate)."""
        updated = self._updated_at.get(key)
        if updated is None:
            return True
        return (time.monotonic() - updated) > self._stale_threshold
