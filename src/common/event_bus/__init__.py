"""Process-local event delivery; network transports belong in adapters."""

from collections import defaultdict
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Callable


@dataclass(frozen=True)
class Event:
    topic: str
    vehicle_id: str | None = None
    payload: dict = field(default_factory=dict)
    timestamp: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat())


class EventBus:
    def __init__(self):
        self._listeners = defaultdict(list)

    def subscribe(self, topic: str, callback: Callable[[Event], None]):
        self._listeners[topic].append(callback)

        def unsubscribe():
            if callback in self._listeners[topic]:
                self._listeners[topic].remove(callback)

        return unsubscribe

    def publish(self, event: Event):
        # Callbacks run synchronously; failures propagate to the caller.
        for callback in tuple(self._listeners[event.topic]):
            callback(event)
