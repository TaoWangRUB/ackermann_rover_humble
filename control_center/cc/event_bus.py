"""Simple async event bus for inter-component communication."""
import asyncio
import logging
from collections import defaultdict
from typing import Any, Callable, Coroutine

logger = logging.getLogger(__name__)

Callback = Callable[..., Coroutine[Any, Any, None]]


class EventBus:
    """Publish/subscribe event bus using asyncio."""

    def __init__(self) -> None:
        self._subscribers: dict[str, list[Callback]] = defaultdict(list)

    def subscribe(self, event: str, callback: Callback) -> None:
        self._subscribers[event].append(callback)

    def unsubscribe(self, event: str, callback: Callback) -> None:
        self._subscribers[event].remove(callback)

    async def emit(self, event: str, **kwargs: Any) -> None:
        for cb in self._subscribers.get(event, []):
            try:
                await cb(**kwargs)
            except Exception:
                logger.exception("Error in handler for event %s", event)
