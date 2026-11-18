"""Health states distinguish measurements, deliberate fusion changes and missing evidence."""

import unittest
from dataclasses import replace

from common.interfaces.health import HealthState
from common.interfaces.replay import ReplayAidingEvidence, ReplayObservation, ReplayRecorderHealth
from common.lifecycle.sensor_health import observed_health


class SensorHealthTests(unittest.TestCase):
    def setUp(self) -> None:
        self.evidence = ReplayAidingEvidence(
            received_monotonic=10,
            gps_requested_enabled=True,
            gps_fix_type=3,
            gnss_xy_active=True,
            vision_xy_active=True,
            visual_pose_received_monotonic=10,
            visual_pose_healthy=True,
        )
        self.observation = ReplayObservation(
            "atlas",
            "Atlas",
            "run-1",
            1,
            None,
            None,
            ReplayRecorderHealth(True),
            self.evidence,
        )

    def health(self, **changes) -> dict:
        observation = replace(self.observation, aiding=replace(self.evidence, **changes))
        return {
            item.system: item
            for item in observed_health(
                observation,
                True,
                10.1,
                visual_supported=True,
            )
        }

    def test_intentionally_disabled_fusion_is_separate_from_gps_hardware_fix(self) -> None:
        rows = self.health(gps_requested_enabled=False, gnss_xy_active=False)
        self.assertEqual(rows["GPS fix"].state, HealthState.HEALTHY)
        self.assertEqual(rows["GPS fusion"].state, HealthState.HEALTHY)
        self.assertIn("intentionally disabled", rows["GPS fusion"].reason)
        waiting = self.health(gps_requested_enabled=False)
        self.assertEqual(waiting["GPS fusion"].state, HealthState.DEGRADED)
        failed = self.health(gps_requested_enabled=False, gps_fix_type=0, gnss_xy_active=False)
        self.assertEqual(failed["GPS fix"].state, HealthState.UNAVAILABLE)
        self.assertEqual(failed["GPS fusion"].state, HealthState.HEALTHY)

    def test_stale_link_or_source_cannot_claim_healthy_or_confirmed_failure(self) -> None:
        for connected, evidence in (
            (False, self.evidence),
            (
                True,
                replace(
                    self.evidence,
                    received_monotonic=8,
                ),
            ),
            (True, replace(self.evidence, source_samples_fresh=False)),
        ):
            rows = observed_health(
                replace(self.observation, aiding=evidence), connected, 10.1, visual_supported=True
            )
            self.assertTrue(all(item.state == HealthState.UNKNOWN for item in rows[:3]))

    def test_stale_vision_does_not_use_fresh_px4_pose_as_camera_proof(self) -> None:
        rows = self.health(visual_pose_received_monotonic=8)
        self.assertEqual(rows["Vision fusion"].state, HealthState.UNKNOWN)
        self.assertEqual(rows["GPS fix"].state, HealthState.HEALTHY)

    def test_missing_sensor_and_reset_do_not_inherit_old_status(self) -> None:
        reset = replace(self.observation, controller_generation="run-2", aiding=None)
        rows = observed_health(reset, True, 10.1, visual_supported=False)
        self.assertEqual(rows[0].state, HealthState.UNKNOWN)
        self.assertEqual(rows[2].state, HealthState.UNSUPPORTED)
        self.assertTrue(all(item.controller_generation == "run-2" for item in rows))
        peer = replace(self.observation, vehicle_id="boreal")
        peer_rows = observed_health(peer, True, 10.1, visual_supported=True)
        self.assertEqual(peer_rows[0].state, HealthState.HEALTHY)
        self.assertEqual(peer_rows[0].vehicle_id, "boreal")
