#!/usr/bin/env python3
"""Control Center — main entry point.

Starts MQTT receiver, telemetry cache, InfluxDB writer, alert notifier,
dashboard server, command gateway, and ACK tracker.
"""
import asyncio
import logging
import signal
import sys
from pathlib import Path

import yaml

from cc.event_bus import EventBus
from cc.mqtt_receiver import MQTTReceiver
from cc.telemetry_cache import TelemetryCache
from cc.influxdb_writer import InfluxDBWriter
from cc.alert_notifier import AlertNotifier
from cc.dashboard_server import DashboardServer
from cc.cmd_gateway import CommandGateway
from cc.safety_gate import SafetyGate
from cc.ack_tracker import AckTracker

logger = logging.getLogger("control_center")


def load_config(path: str = "config/control_center.yaml") -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


async def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    )

    config_path = sys.argv[1] if len(sys.argv) > 1 else "config/control_center.yaml"
    cfg = load_config(config_path)
    logger.info("Loaded config from %s", config_path)

    bus = EventBus()

    # Build components
    cache = TelemetryCache(bus, cfg.get("telemetry_cache", {}))
    mqtt_rx = MQTTReceiver(bus, cfg.get("mqtt", {}))
    influx = InfluxDBWriter(bus, cfg.get("influxdb", {}))
    notifier = AlertNotifier(bus, cfg.get("notifier", {}))
    safety = SafetyGate(cache, cfg.get("cmd_gateway", {}))
    gateway = CommandGateway(bus, mqtt_rx, safety, cfg.get("cmd_gateway", {}))
    ack_tracker = AckTracker(bus, cfg.get("cmd_gateway", {}))
    dashboard = DashboardServer(bus, cache, gateway, ack_tracker, cfg.get("dashboard", {}))

    # Graceful shutdown
    loop = asyncio.get_running_loop()
    stop_event = asyncio.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop_event.set)

    # Start
    await mqtt_rx.start()
    await influx.start()
    await dashboard.start()

    logger.info("Control Center running — press Ctrl+C to stop")
    await stop_event.wait()

    # Shutdown
    logger.info("Shutting down...")
    await dashboard.stop()
    await influx.stop()
    await mqtt_rx.stop()
    logger.info("Control Center stopped")


if __name__ == "__main__":
    asyncio.run(main())
