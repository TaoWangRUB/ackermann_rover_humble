"""CC-4: Alert Notifier — deduplication + multi-channel notifications."""
import asyncio
import logging
import time
from typing import Any

from cc.event_bus import EventBus

logger = logging.getLogger(__name__)

SEVERITY_ORDER = {"WARN": 0, "ERROR": 1, "CRITICAL": 2}


class AlertNotifier:
    """Dispatches alert notifications with per-alert cooldown deduplication."""

    def __init__(self, bus: EventBus, cfg: dict[str, Any]) -> None:
        self._bus = bus
        self._cooldown_s = cfg.get("default_cooldown_s", 60)
        self._channels_cfg = cfg.get("channels", {})
        self._last_notified: dict[str, float] = {}

        bus.subscribe("evt.alert", self._on_alert)

    async def _on_alert(self, data: Any = None, **kwargs: Any) -> None:
        now = time.monotonic()
        for alert_id in data.active_alerts:
            # Dedup check
            last = self._last_notified.get(alert_id, 0)
            if now - last < self._cooldown_s:
                continue
            self._last_notified[alert_id] = now

            severity = self._classify_severity(alert_id)
            await self._dispatch(alert_id, severity)

    def _classify_severity(self, alert_id: str) -> str:
        error_alerts = {
            "CAM_DISCONNECTED", "PX4_HEARTBEAT_LOST", "PX4_BATTERY_CRITICAL",
            "JETSON_CPU_HOT", "HW_THROTTLE_THERMAL", "HW_THROTTLE_POWER",
            "JETSON_DISK_LOW",
        }
        return "ERROR" if alert_id in error_alerts else "WARN"

    async def _dispatch(self, alert_id: str, severity: str) -> None:
        for name, ch_cfg in self._channels_cfg.items():
            if not ch_cfg.get("enabled", False):
                continue
            min_sev = ch_cfg.get("min_severity", "WARN")
            if SEVERITY_ORDER.get(severity, 0) < SEVERITY_ORDER.get(min_sev, 0):
                continue

            try:
                if name == "terminal":
                    await self._notify_terminal(alert_id, severity)
                elif name == "sound":
                    await self._notify_sound(alert_id, severity, ch_cfg)
                elif name == "desktop":
                    await self._notify_desktop(alert_id, severity)
                elif name == "slack":
                    await self._notify_slack(alert_id, severity, ch_cfg)
                elif name == "email":
                    await self._notify_email(alert_id, severity, ch_cfg)
            except Exception:
                logger.exception("Notification channel %s failed for %s", name, alert_id)

    async def _notify_terminal(self, alert_id: str, severity: str) -> None:
        icon = "\u26a0\ufe0f" if severity == "WARN" else "\u274c"
        logger.warning("%s [%s] %s", icon, severity, alert_id)

    async def _notify_sound(self, alert_id: str, severity: str,
                            cfg: dict[str, Any]) -> None:
        wav = cfg.get("settings", {}).get("wav_file", "alert.wav")
        try:
            proc = await asyncio.create_subprocess_exec(
                "aplay", "-q", wav,
                stdout=asyncio.subprocess.DEVNULL,
                stderr=asyncio.subprocess.DEVNULL)
            await proc.wait()
        except FileNotFoundError:
            logger.debug("aplay not found — sound notification skipped")

    async def _notify_desktop(self, alert_id: str, severity: str) -> None:
        try:
            from plyer import notification
            notification.notify(
                title=f"Rover Alert [{severity}]",
                message=alert_id,
                timeout=5)
        except ImportError:
            logger.debug("plyer not installed — desktop notification skipped")

    async def _notify_slack(self, alert_id: str, severity: str,
                            cfg: dict[str, Any]) -> None:
        settings = cfg.get("settings", {})
        webhook_url = settings.get("webhook_url", "")
        mention = settings.get("mention", "")
        if not webhook_url:
            return
        try:
            import aiohttp
            text = f"{mention} [{severity}] {alert_id}" if mention else f"[{severity}] {alert_id}"
            async with aiohttp.ClientSession() as session:
                await session.post(webhook_url, json={"text": text})
        except Exception:
            logger.exception("Slack notification failed")

    async def _notify_email(self, alert_id: str, severity: str,
                            cfg: dict[str, Any]) -> None:
        settings = cfg.get("settings", {})
        host = settings.get("smtp_host", "")
        if not host:
            return
        try:
            import smtplib
            from email.message import EmailMessage
            msg = EmailMessage()
            msg["Subject"] = f"Rover Alert [{severity}]: {alert_id}"
            msg["From"] = settings.get("from_addr", "")
            msg["To"] = ", ".join(settings.get("to_addrs", []))
            msg.set_content(f"Alert: {alert_id}\nSeverity: {severity}")

            loop = asyncio.get_running_loop()
            await loop.run_in_executor(None, lambda: _send_email(
                host, settings.get("smtp_port", 587), msg))
        except Exception:
            logger.exception("Email notification failed")


def _send_email(host: str, port: int, msg: Any) -> None:
    with smtplib.SMTP(host, port) as server:
        server.starttls()
        server.send_message(msg)
