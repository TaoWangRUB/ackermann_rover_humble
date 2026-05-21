#!/usr/bin/env python3
"""Lightweight MAVLink serial↔UDP bridge with GCS heartbeat.

Drop-in replacement for MAVProxy when only forwarding is needed.
Typical CPU usage: <2% vs MAVProxy's 80%+.

Usage:
    python3 scripts/mavlink_bridge.py \\
        --serial /dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00 \\
        --baud 57600 --out 127.0.0.1:14550 --out 127.0.0.1:14551
"""
import argparse
import io
import logging
import select
import signal
import socket
import sys
import time

import serial
from pymavlink import mavutil

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s [mavlink_bridge] %(message)s",
)
log = logging.getLogger("mavlink_bridge")


def build_gcs_heartbeat() -> bytes:
    """Build a MAVLink GCS heartbeat using pymavlink."""
    buf = io.BytesIO()
    mav = mavutil.mavlink.MAVLink(buf, srcSystem=255, srcComponent=0)
    mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )
    return buf.getvalue()


def main() -> int:
    parser = argparse.ArgumentParser(description="Lightweight MAVLink serial↔UDP bridge")
    parser.add_argument("--serial", required=True, help="Serial device path")
    parser.add_argument("--baud", type=int, default=57600, help="Serial baud rate")
    parser.add_argument("--out", action="append", default=None,
                        help="UDP target host:port (repeatable for fan-out)")
    parser.add_argument("--heartbeat-interval", type=float, default=1.0,
                        help="GCS heartbeat interval in seconds")
    args = parser.parse_args()

    # Parse UDP targets (default to 14550 if none given)
    out_list = args.out or ["127.0.0.1:14550"]
    udp_targets = []
    for ep in out_list:
        h, p = ep.rsplit(":", 1)
        udp_targets.append((h, int(p)))

    # Open serial with DTR set (required for Cube Black USB CDC)
    try:
        ser = serial.Serial(args.serial, args.baud, timeout=0)
        ser.setDTR(True)
        log.info("Serial opened: %s @ %d baud", args.serial, args.baud)
    except Exception:
        log.exception("Failed to open serial port %s", args.serial)
        return 1

    # UDP socket for sending (outbound to listeners) and receiving (inbound from tools)
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.setblocking(False)
    # Bind to an ephemeral port — we send TO udp_target, listeners bind there.
    udp.bind(("0.0.0.0", 0))
    bound_port = udp.getsockname()[1]
    for t in udp_targets:
        log.info("UDP target: %s:%d", t[0], t[1])
    log.info("Bound on ephemeral port %d", bound_port)

    # Pre-build heartbeat
    gcs_hb = build_gcs_heartbeat()
    last_hb = 0.0
    hb_interval = args.heartbeat_interval

    # Stats
    serial_bytes = 0
    udp_bytes = 0
    last_stats = time.monotonic()

    running = True
    def _stop(sig, frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    log.info("Bridge running — Ctrl+C to stop")

    while running:
        # select on serial fd and UDP socket, 100ms timeout
        readable, _, _ = select.select([ser, udp], [], [], 0.1)

        for fd in readable:
            if fd is ser:
                data = ser.read(ser.in_waiting or 1)
                if data:
                    for t in udp_targets:
                        try:
                            udp.sendto(data, t)
                        except OSError:
                            pass  # UDP send failure (no listener) — harmless
                    serial_bytes += len(data)
            elif fd is udp:
                try:
                    data, addr = udp.recvfrom(2048)
                    if data:
                        ser.write(data)
                        udp_bytes += len(data)
                except OSError:
                    pass

        # GCS heartbeat
        now = time.monotonic()
        if now - last_hb >= hb_interval:
            try:
                ser.write(gcs_hb)
            except OSError:
                log.warning("Serial write failed for heartbeat")
            last_hb = now

        # Periodic stats (every 30s)
        if now - last_stats >= 30.0:
            log.info("Stats: serial→udp %d bytes, udp→serial %d bytes",
                     serial_bytes, udp_bytes)
            serial_bytes = udp_bytes = 0
            last_stats = now

    log.info("Shutting down")
    ser.close()
    udp.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
