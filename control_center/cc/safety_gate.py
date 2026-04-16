"""CC-6a: Safety Gate — per-CommandType precondition checks."""
import logging
from typing import Any

from cc.telemetry_cache import TelemetryCache

logger = logging.getLogger(__name__)


class SafetyGate:
    """Validates command preconditions using telemetry cache state."""

    def __init__(self, cache: TelemetryCache, cfg: dict[str, Any]) -> None:
        self._cache = cache
        self._cfg = cfg

    async def check(self, cmd_type: str, params: dict[str, Any] | None = None) -> tuple[bool, str]:
        """Return (allowed, reason). Check preconditions for given command type."""
        # E-STOP always passes
        if cmd_type == "estop":
            return True, "E-STOP always allowed"

        # Check cache staleness
        health_data, health_stale = await self._cache.get("health")

        if cmd_type == "arm":
            return await self._check_arm(health_data, health_stale)
        elif cmd_type == "disarm":
            return True, "Disarm always allowed"
        elif cmd_type == "nav_goal":
            return await self._check_nav_goal(health_data, health_stale)
        elif cmd_type == "set_mode":
            return await self._check_set_mode(health_data, health_stale)
        elif cmd_type == "cancel_goal":
            return True, "Cancel goal always allowed"
        elif cmd_type == "drive":
            return await self._check_drive(health_data, health_stale)

        return False, f"Unknown command type: {cmd_type}"

    async def _check_arm(self, health: Any, stale: bool) -> tuple[bool, str]:
        if stale or health is None:
            return False, "Telemetry stale — cannot arm"

        px4 = health.px4
        if not px4.connected:
            return False, "PX4 not connected — cannot arm"
        if not px4.armable:
            return False, "PX4 preflight checks failed — cannot arm"

        return True, "Arm preconditions met"

    async def _check_nav_goal(self, health: Any, stale: bool) -> tuple[bool, str]:
        if stale or health is None:
            return False, "Telemetry stale — cannot navigate"

        px4 = health.px4
        if not px4.connected:
            return False, "PX4 not connected — cannot navigate"
        if not px4.armed:
            return False, "Not armed — cannot navigate"
        if health.slam_latency_ms > 200.0 and health.slam_latency_ms >= 0:
            return False, f"map→odom TF age too high ({health.slam_latency_ms:.0f}ms) — cannot navigate"
        if health.overall_health == "ERROR":
            return False, "Overall health ERROR — cannot navigate"
        if not health.nav2.available:
            return False, "Nav2 not ready — cannot navigate"

        return True, "Nav goal preconditions met"

    async def _check_set_mode(self, health: Any, stale: bool) -> tuple[bool, str]:
        if stale or health is None:
            return False, "Telemetry stale — cannot change mode"

        if not health.px4.connected:
            return False, "PX4 not connected — cannot change mode"

        return True, "Set mode preconditions met"

    async def _check_drive(self, health: Any, stale: bool) -> tuple[bool, str]:
        if stale or health is None:
            return False, "Telemetry stale — cannot drive"

        px4 = health.px4
        if not px4.connected:
            return False, "PX4 not connected — cannot drive"
        if not px4.armed:
            return False, "Not armed — cannot drive"

        return True, "Drive preconditions met"
