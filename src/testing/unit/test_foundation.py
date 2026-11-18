"""Check configuration, event, registry and lifecycle foundation contracts."""

import unittest

from common.event_bus import Event, EventBus
from common.lifecycle import Lifecycle, State
from common.simulation_manager import MockSimulation
from common.vehicle_manager import Vehicle, VehicleRegistry
from control_panel.panel import PanelRegistry, PanelSpec


class FoundationTests(unittest.TestCase):
    def test_duplicate_vehicle_address_rejected(self):
        registry = VehicleRegistry()
        registry.add(Vehicle("one"))
        with self.assertRaises(ValueError):
            registry.add(Vehicle("two"))
        self.assertEqual(len(registry.all()), 1)

    def test_lifecycle_transitions_and_event_delivery(self):
        bus, received = EventBus(), []
        unsubscribe = bus.subscribe("lifecycle.changed", received.append)
        lifecycle = Lifecycle(MockSimulation(), bus)
        with self.assertRaises(ValueError):
            lifecycle.execute("pause")
        for operation in ("start", "pause", "start", "stop", "reset"):
            lifecycle.execute(operation)
        self.assertEqual(lifecycle.state, State.STOPPED)
        self.assertEqual(len(received), 5)
        unsubscribe()
        bus.publish(Event("lifecycle.changed"))
        self.assertEqual(len(received), 5)

    def test_failed_adapter_does_not_report_success(self):
        class Failed(MockSimulation):
            def start(self):
                raise RuntimeError("unavailable")

        lifecycle = Lifecycle(Failed(), EventBus())
        with self.assertRaises(RuntimeError):
            lifecycle.execute("start")
        self.assertEqual(lifecycle.state, State.ERROR)
        lifecycle.execute("reset")
        self.assertEqual(lifecycle.state, State.STOPPED)

    def test_panel_registration_is_independent_of_qt(self):
        registry = PanelRegistry()
        spec = PanelSpec("telemetry", "Telemetry", lambda: None)
        registry.register(spec)
        with self.assertRaises(ValueError):
            registry.register(spec)
        self.assertEqual(registry.all(), (spec,))
