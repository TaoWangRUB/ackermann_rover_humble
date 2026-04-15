"""CC-3: InfluxDB Writer — async batch writes for telemetry, sync for alerts/commands."""
import asyncio
import logging
from typing import Any

from cc.event_bus import EventBus

logger = logging.getLogger(__name__)

try:
    from influxdb_client import InfluxDBClient, Point
    from influxdb_client.client.write_api import ASYNCHRONOUS
    HAS_INFLUX = True
except ImportError:
    HAS_INFLUX = False
    logger.warning("influxdb-client not installed — InfluxDB writing disabled")


class InfluxDBWriter:
    """Writes rover telemetry to InfluxDB 2.x."""

    def __init__(self, bus: EventBus, cfg: dict[str, Any]) -> None:
        self._bus = bus
        self._url = cfg.get("url", "http://localhost:8086")
        self._token = cfg.get("token", "rover-dev-token")
        self._org = cfg.get("org", "rover")
        self._bucket = cfg.get("bucket", "rover_telemetry")
        self._batch_size = cfg.get("batch_size", 50)
        self._flush_interval = cfg.get("flush_interval_ms", 2000)
        self._max_retries = cfg.get("max_retries", 3)
        self._client: Any = None
        self._write_api: Any = None

        bus.subscribe("evt.health", self._on_health)
        bus.subscribe("evt.alert", self._on_alert)
        bus.subscribe("evt.cmd_ack", self._on_cmd_ack)

    async def start(self) -> None:
        if not HAS_INFLUX:
            logger.warning("InfluxDB writer disabled (missing library)")
            return
        try:
            self._client = InfluxDBClient(
                url=self._url, token=self._token, org=self._org)
            self._write_api = self._client.write_api(
                write_options=ASYNCHRONOUS)
            logger.info("InfluxDB writer started (%s)", self._url)
        except Exception:
            logger.exception("InfluxDB connection failed — writes disabled")
            self._client = None

    async def stop(self) -> None:
        if self._write_api:
            try:
                self._write_api.close()
            except Exception:
                pass
        if self._client:
            self._client.close()
        logger.info("InfluxDB writer stopped")

    async def _on_health(self, data: Any = None, **kwargs: Any) -> None:
        if not self._write_api:
            return
        try:
            p = Point("rover_telemetry")
            p.tag("source", "aggregator")

            # Camera fields (one point per camera)
            for cam in data.cameras:
                cam_p = Point("rover_cam_telemetry")
                cam_p.tag("source", "aggregator")
                cam_p.tag("camera_id", cam.camera_id or "unknown")
                cam_p.field("cam_connected", cam.connected)
                cam_p.field("cam_frame_delta_ms", cam.frame_delta_ms)
                cam_p.field("cam_depth_fps", cam.depth_fps)
                self._write_api.write(bucket=self._bucket, record=cam_p)

            # PX4 fields
            if data.HasField("px4"):
                px4 = data.px4
                p.field("px4_connected", px4.connected)
                p.field("px4_armed", px4.armed)
                p.field("px4_battery_v", px4.battery_voltage_v)
                p.field("px4_battery_pct", px4.battery_remaining_pct)
                p.field("px4_heartbeat_age_ms", px4.heartbeat_age_ms)

            # Jetson fields
            if data.HasField("jetson"):
                jet = data.jetson
                if jet.cpu_usage_pct:
                    avg_cpu = sum(jet.cpu_usage_pct) / len(jet.cpu_usage_pct)
                    p.field("jetson_cpu_pct", avg_cpu)
                p.field("jetson_gpu_pct", jet.gpu_usage_pct)
                p.field("jetson_ram_used_mb", jet.ram_used_mb)
                p.field("jetson_temp_cpu_c", jet.temp_cpu_c)
                p.field("jetson_temp_gpu_c", jet.temp_gpu_c)
                p.field("jetson_disk_free_gb", jet.disk_free_gb)

            p.field("slam_latency_ms", data.slam_latency_ms)
            p.field("overall_health", data.overall_health)

            self._write_api.write(bucket=self._bucket, record=p)
        except Exception:
            logger.exception("InfluxDB telemetry write failed")

    async def _on_alert(self, data: Any = None, **kwargs: Any) -> None:
        if not self._write_api:
            return
        try:
            for alert_id in data.active_alerts:
                p = Point("rover_alerts")
                p.tag("alert_id", alert_id)
                p.field("severity", "ERROR" if "CRITICAL" in alert_id or
                        "DISCONNECTED" in alert_id or "LOST" in alert_id
                        else "WARN")
                p.field("timestamp_ms", data.timestamp)
                self._write_api.write(bucket=self._bucket, record=p)
        except Exception:
            logger.exception("InfluxDB alert write failed")

    async def _on_cmd_ack(self, data: Any = None, **kwargs: Any) -> None:
        if not self._write_api:
            return
        try:
            p = Point("rover_commands")
            p.tag("cmd_id", data.cmd_id)
            p.field("cmd_type", data.cmd_type)
            p.field("ack_status", data.status)
            p.field("message", data.message)
            p.field("round_trip_ms", data.round_trip_ms)
            self._write_api.write(bucket=self._bucket, record=p)
        except Exception:
            logger.exception("InfluxDB command write failed")
