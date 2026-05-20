#!/usr/bin/env python3
"""PX4 hardware monitor — reads CPU load and RAM via MAVLink, publishes to MQTT.

Listens for MAVLink SYS_STATUS (CPU load) and MEMINFO (RAM) from MAVProxy,
publishes JSON to MQTT topic ``rover/health/px4_hw``.

Requires: pymavlink, paho-mqtt  (both already on the Jetson host)

Usage:
    python3 scripts/px4_hw_monitor.py \\
        --mavlink udpin:0.0.0.0:14550 \\
        --mqtt-host 192.168.0.248
"""
import argparse
import json
import logging
import signal
import sys
import time

from pymavlink import mavutil

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s [px4_hw] %(message)s",
)
log = logging.getLogger("px4_hw_monitor")

TOPIC = "rover/health/px4_hw"
DEFAULT_INTERVAL = 2.0


def main() -> int:
    parser = argparse.ArgumentParser(description="PX4 HW monitor (MAVLink → MQTT)")
    parser.add_argument(
        "--mavlink", default="udpin:0.0.0.0:14550",
        help="MAVLink connection string (default: udpin:0.0.0.0:14550)")
    parser.add_argument(
        "--mqtt-host", default="localhost",
        help="MQTT broker host")
    parser.add_argument("--mqtt-port", type=int, default=1883)
    parser.add_argument(
        "--interval", type=float, default=DEFAULT_INTERVAL,
        help="Publish interval in seconds (default: 2)")
    args = parser.parse_args()

    # ── MQTT ──────────────────────────────────────────────────────────
    try:
        import paho.mqtt.client as paho
    except ImportError:
        log.error("paho-mqtt not installed: pip3 install paho-mqtt")
        return 1

    mqtt = paho.Client(client_id="px4_hw_monitor", protocol=paho.MQTTv311)
    try:
        mqtt.connect(args.mqtt_host, args.mqtt_port, keepalive=60)
    except Exception:
        log.exception("MQTT connect failed to %s:%d", args.mqtt_host, args.mqtt_port)
        return 1
    mqtt.loop_start()
    log.info("MQTT connected to %s:%d", args.mqtt_host, args.mqtt_port)

    # ── MAVLink ───────────────────────────────────────────────────────
    log.info("Connecting to MAVLink at %s …", args.mavlink)
    mav = mavutil.mavlink_connection(
        args.mavlink, source_system=254, source_component=191)

    hb = mav.wait_heartbeat(timeout=10)
    if hb:
        log.info("Heartbeat from sysid=%d compid=%d",
                 mav.target_system, mav.target_component)
    else:
        log.warning("No heartbeat — will continue waiting")

    # Request EXTENDED_STATUS stream (includes SYS_STATUS + MEMINFO) at 1 Hz
    if mav.target_system:
        mav.mav.request_data_stream_send(
            mav.target_system, mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
            1,   # 1 Hz — minimal load on Cube Black
            1)   # start

    # ── State ─────────────────────────────────────────────────────────
    cpu_load_pct: float | None = None
    ram_usage_pct: float | None = None
    last_publish = 0.0
    running = True

    def _stop(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    while running:
        msg = mav.recv_match(blocking=True, timeout=1.0)
        if msg is not None:
            mtype = msg.get_type()
            if mtype == "SYS_STATUS":
                # load: 0-1000 (permille of mainloop time)
                cpu_load_pct = msg.load / 10.0
            elif mtype == "MEMINFO":
                # freemem32: bytes free (uint32); freemem: uint16 (legacy)
                free_b = msg.freemem32 if hasattr(msg, "freemem32") and msg.freemem32 > 0 else msg.freemem
                if free_b > 0:
                    # Cube Black total SRAM ~256 KB
                    TOTAL_RAM = 256 * 1024
                    used = max(0, TOTAL_RAM - free_b)
                    ram_usage_pct = (used / TOTAL_RAM) * 100.0

        now = time.monotonic()
        if now - last_publish >= args.interval and cpu_load_pct is not None:
            payload: dict = {
                "cpu_load_pct": round(cpu_load_pct, 1),
                "timestamp": int(time.time() * 1000),
            }
            if ram_usage_pct is not None:
                payload["ram_usage_pct"] = round(ram_usage_pct, 1)
            mqtt.publish(TOPIC, json.dumps(payload), qos=0)
            log.debug("Published: %s", payload)
            last_publish = now

    log.info("Shutting down")
    mav.close()
    mqtt.loop_stop()
    mqtt.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
