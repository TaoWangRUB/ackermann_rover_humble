"""CC-7: ACK Tracker — track pending commands and resolve on ACK."""
import asyncio
import logging
import time
from typing import Any

from cc.event_bus import EventBus

logger = logging.getLogger(__name__)


class AckTracker:
    """Tracks pending commands and resolves futures on matching CommandAck."""

    def __init__(self, bus: EventBus, cfg: dict[str, Any]) -> None:
        self._bus = bus
        self._timeout_s = cfg.get("ack_timeout_s", 5.0)
        self._pending: dict[str, dict[str, Any]] = {}
        self._history: list[dict[str, Any]] = []

        bus.subscribe("evt.cmd_sent", self._on_cmd_sent)
        bus.subscribe("evt.cmd_ack", self._on_cmd_ack)

    async def _on_cmd_sent(self, cmd_id: str = "", cmd_type: str = "",
                           timestamp: int = 0, **kwargs: Any) -> None:
        future = asyncio.get_running_loop().create_future()
        self._pending[cmd_id] = {
            "future": future,
            "cmd_type": cmd_type,
            "sent_at": time.monotonic(),
            "sent_ts": timestamp,
        }

        # Start timeout watcher
        asyncio.create_task(self._watch_timeout(cmd_id))

    async def _on_cmd_ack(self, data: Any = None, **kwargs: Any) -> None:
        cmd_id = data.cmd_id
        entry = self._pending.pop(cmd_id, None)
        if entry is None:
            logger.debug("ACK for unknown cmd_id=%s (may have timed out)", cmd_id)
            return

        round_trip = int((time.monotonic() - entry["sent_at"]) * 1000)
        status_name = _ack_status_name(data.status)

        logger.info("ACK received: cmd_id=%s status=%s rt=%dms msg=%s",
                     cmd_id, status_name, round_trip, data.message)

        record = {
            "cmd_id": cmd_id,
            "cmd_type": entry["cmd_type"],
            "ack_status": status_name,
            "message": data.message,
            "round_trip_ms": round_trip,
        }
        self._history.append(record)

        # Resolve future
        if not entry["future"].done():
            entry["future"].set_result(record)

        # Broadcast to WS clients
        await self._bus.emit("evt.cmd_ack_resolved", **record)

    async def _watch_timeout(self, cmd_id: str) -> None:
        await asyncio.sleep(self._timeout_s)
        entry = self._pending.pop(cmd_id, None)
        if entry is None:
            return  # Already resolved

        logger.warning("ACK timeout for cmd_id=%s", cmd_id)
        record = {
            "cmd_id": cmd_id,
            "cmd_type": entry["cmd_type"],
            "ack_status": "ACK_FAILED",
            "message": "ACK timeout",
            "round_trip_ms": int(self._timeout_s * 1000),
        }
        self._history.append(record)

        if not entry["future"].done():
            entry["future"].set_result(record)

    def get_history(self) -> list[dict[str, Any]]:
        return list(self._history)


def _ack_status_name(status: int) -> str:
    names = {0: "ACK_RECEIVED", 1: "ACK_ACCEPTED", 2: "ACK_REJECTED",
             3: "ACK_COMPLETED", 4: "ACK_FAILED"}
    return names.get(status, f"UNKNOWN({status})")
