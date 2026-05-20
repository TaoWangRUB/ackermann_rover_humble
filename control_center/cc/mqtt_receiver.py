"""CC-1: MQTT Receiver — subscribes to rover telemetry and command ACK topics."""
import asyncio
import logging
import time
from typing import Any

import paho.mqtt.client as paho

from cc.event_bus import EventBus

logger = logging.getLogger(__name__)

# Protobuf imports (generated from proto/rover_health.proto)
try:
    from proto import rover_health_pb2 as pb
except ImportError:
    pb = None
    logger.warning("rover_health_pb2 not found — run protoc to generate")


class MQTTReceiver:
    """Async MQTT receiver with exponential backoff reconnection."""

    def __init__(self, bus: EventBus, cfg: dict[str, Any]) -> None:
        self._bus = bus
        self._host = cfg.get("broker_host", "localhost")
        self._port = cfg.get("broker_port", 1883)
        self._client_id = cfg.get("client_id", "control_center_01")
        self._reconnect_min = cfg.get("reconnect_min_s", 1)
        self._reconnect_max = cfg.get("reconnect_max_s", 30)
        self._subscriptions = cfg.get("subscriptions", [])

        self._client = paho.Client(client_id=self._client_id, protocol=paho.MQTTv311)
        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        self._client.on_message = self._on_message

        self._loop: asyncio.AbstractEventLoop | None = None
        self._backoff = self._reconnect_min
        self._connected = False

    @property
    def client(self) -> paho.Client:
        return self._client

    async def start(self) -> None:
        self._loop = asyncio.get_running_loop()
        try:
            self._client.connect(self._host, self._port, keepalive=60)
            self._client.loop_start()
            logger.info("MQTT connecting to %s:%d", self._host, self._port)
        except Exception:
            logger.exception("MQTT initial connect failed — will retry")
            self._schedule_reconnect()

    async def stop(self) -> None:
        self._client.loop_stop()
        self._client.disconnect()
        logger.info("MQTT receiver stopped")

    def publish(self, topic: str, payload: bytes, qos: int = 0) -> None:
        self._client.publish(topic, payload, qos=qos)

    def _on_connect(self, client: Any, userdata: Any, flags: Any, rc: int) -> None:
        if rc == 0:
            logger.info("MQTT connected")
            self._connected = True
            self._backoff = self._reconnect_min
            for sub in self._subscriptions:
                client.subscribe(sub["topic"], sub.get("qos", 0))
                logger.info("  subscribed: %s (QoS %d)", sub["topic"], sub.get("qos", 0))
            if self._loop:
                asyncio.run_coroutine_threadsafe(
                    self._bus.emit("evt.connection_restored"), self._loop)
        else:
            logger.warning("MQTT connect failed rc=%d", rc)

    def _on_disconnect(self, client: Any, userdata: Any, rc: int) -> None:
        self._connected = False
        if rc != 0:
            logger.warning("MQTT unexpected disconnect rc=%d — reconnecting", rc)
            if self._loop:
                asyncio.run_coroutine_threadsafe(
                    self._bus.emit("evt.connection_lost"), self._loop)
            self._schedule_reconnect()

    def _schedule_reconnect(self) -> None:
        delay = self._backoff
        self._backoff = min(self._backoff * 2, self._reconnect_max)
        logger.info("Reconnecting in %ds", delay)
        try:
            time.sleep(0.1)  # small delay before retry
            self._client.reconnect()
        except Exception:
            logger.warning("Reconnect failed, next attempt in %ds", self._backoff)

    def _on_message(self, client: Any, userdata: Any, msg: paho.MQTTMessage) -> None:
        if not self._loop or pb is None:
            return

        topic = msg.topic
        payload = msg.payload

        try:
            if topic == "rover/health/overall":
                health = pb.RoverHealth()
                health.ParseFromString(payload)
                asyncio.run_coroutine_threadsafe(
                    self._bus.emit("evt.health", data=health), self._loop)
                # Also emit sub-component events
                for cam in health.cameras:
                    asyncio.run_coroutine_threadsafe(
                        self._bus.emit("evt.health.cam", data=cam), self._loop)
                asyncio.run_coroutine_threadsafe(
                    self._bus.emit("evt.health.px4", data=health.px4), self._loop)
                asyncio.run_coroutine_threadsafe(
                    self._bus.emit("evt.health.jetson", data=health.jetson), self._loop)

            elif topic == "rover/alerts":
                health = pb.RoverHealth()
                health.ParseFromString(payload)
                asyncio.run_coroutine_threadsafe(
                    self._bus.emit("evt.alert", data=health), self._loop)

            elif topic == "rover/health/px4_hw":
                import json as _json
                hw = _json.loads(payload)
                asyncio.run_coroutine_threadsafe(
                    self._bus.emit("evt.px4_hw", data=hw), self._loop)

            elif topic == "rover/cmd/ack":
                ack = pb.CommandAck()
                ack.ParseFromString(payload)
                asyncio.run_coroutine_threadsafe(
                    self._bus.emit("evt.cmd_ack", data=ack), self._loop)

        except Exception:
            logger.exception("Failed to parse message on %s", topic)
