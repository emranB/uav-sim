from enum import Enum

from common.event_bus import Event, EventBus
from common.interfaces import SimulationAdapter


class State(str, Enum):
    STOPPED = "stopped"
    RUNNING = "running"
    PAUSED = "paused"
    ERROR = "error"


class Lifecycle:
    def __init__(self, adapter: SimulationAdapter, events: EventBus):
        self.adapter, self.events = adapter, events
        self.state = State.STOPPED

    def execute(self, operation: str):
        transitions = {
            "start": ({State.STOPPED, State.PAUSED}, State.RUNNING),
            "pause": ({State.RUNNING}, State.PAUSED),
            "stop": ({State.RUNNING, State.PAUSED, State.ERROR}, State.STOPPED),
            "reset": (set(State), State.STOPPED),
        }
        if operation not in transitions:
            raise ValueError(f"Unknown operation: {operation}")
        allowed, target = transitions[operation]
        if self.state not in allowed:
            raise ValueError(f"Cannot {operation} while {self.state.value}")
        try:
            getattr(self.adapter, operation)()
        except Exception:
            self.state = State.ERROR
            self.events.publish(Event("lifecycle.error", payload={"operation": operation}))
            raise
        self.state = target
        self.events.publish(Event("lifecycle.changed", payload={"state": target.value}))
