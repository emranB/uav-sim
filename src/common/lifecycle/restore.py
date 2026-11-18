"""Serialize safe fleet restore, preserving persisted records and unknown-state failures."""

import json
import threading
import time
import uuid
from pathlib import Path
from typing import TYPE_CHECKING

from common.interfaces.restore import RestoreRuntime
from common.logging.storage import write_json_atomic

if TYPE_CHECKING:
    from common.logging.replay_capture import ReplayCapture


class MasterReset:
    """Restore UAV runtime only after acknowledged recovery and fresh disarm quorum."""

    def __init__(
        self,
        runtime: RestoreRuntime,
        directory: Path,
        replay: "ReplayCapture | None" = None,
        *,
        landing_timeout_seconds: float = 60,
    ) -> None:
        self.runtime, self.directory, self.replay = runtime, directory, replay
        self.landing_timeout_seconds = landing_timeout_seconds
        self._lock = threading.RLock()
        self._cancel = threading.Event()
        self._worker: threading.Thread | None = None
        self._status = {"state": "idle", "active": False}
        directory.mkdir(parents=True, exist_ok=True)
        for path in directory.glob("restore-*.json"):
            previous = json.loads(path.read_text())
            if previous.get("active"):
                write_json_atomic(
                    path,
                    {
                        **previous,
                        "active": False,
                        "state": "interrupted",
                        "error": "Restore owner ended before verification",
                    },
                )

    def snapshot(self) -> dict:
        """Progress includes affected fleet, actual errors and acknowledged generations."""
        with self._lock:
            return json.loads(json.dumps(self._status))

    def availability(self) -> dict:
        return self.runtime.describe()

    def start(self) -> dict:
        with self._lock:
            if self._worker is not None and self._worker.is_alive():
                raise RuntimeError("Master Reset is already in progress")
            description = self.runtime.describe()
            if not description.get("available"):
                raise RuntimeError(
                    description.get("reason", "Required UAV service owners unavailable")
                )
            identities = tuple(description["vehicle_ids"])
            if not identities or len(set(identities)) != len(identities) or len(identities) > 8:
                raise ValueError(
                    "Master Reset requires an explicit configured fleet of 1..8 vehicles"
                )
            self.runtime.enter()
            try:
                self._status = {
                    "restore_id": f"restore-{uuid.uuid4().hex}",
                    "state": "preparing",
                    "active": True,
                    "vehicle_ids": list(identities),
                    "services": description.get("services", []),
                    "events": [],
                    "started_wall_time": time.time(),
                    "error": None,
                }
                self._cancel.clear()
                self._publish()
                self._worker = threading.Thread(
                    target=self._run, args=(identities,), name="master-reset", daemon=False
                )
                self._worker.start()
            except (OSError, RuntimeError):
                self.runtime.leave()
                raise
            return {"accepted": True, **self.snapshot()}

    def cancel(self) -> dict:
        """Request cancellation at the next safe boundary, never interrupt landing."""
        with self._lock:
            if not self._status["active"]:
                raise RuntimeError("No Master Reset is active")
            self._cancel.set()
            self._status["cancellation_requested"] = True
            return {"accepted": True, **self.snapshot()}

    def close(self) -> None:
        """Do not interrupt an airborne landing transition when the API shuts down."""
        worker = self._worker
        if worker is not None and worker.is_alive():
            worker.join(300)
            if worker.is_alive():
                raise RuntimeError("Master Reset remains active; preserve its safe landing owner")

    def _publish(self) -> None:
        write_json_atomic(self.directory / f"{self._status['restore_id']}.json", self._status)

    def _phase(self, state: str, **values) -> None:
        with self._lock:
            self._status.update(state=state, **values)
            self._status["events"].append({"state": state, "wall_time": time.time()})
            self._publish()

    def _run(self, identities: tuple[str, ...]) -> None:
        try:
            self._phase("recovering_links")
            self.runtime.prepare(identities)
            self._phase("landing")
            pending_acknowledgements = set()
            for identity in identities:
                result = self.runtime.land(identity)
                if result.get("accepted") is not True:
                    raise RuntimeError(
                        f"{identity}: landing recovery rejected: {result.get('error')}"
                    )
                if result.get("landing_acknowledgement_pending"):
                    pending_acknowledgements.add(identity)
            self._phase("waiting_disarm")
            deadline = time.monotonic() + self.landing_timeout_seconds
            stable_since = None
            previous_generations = {}
            while True:
                states = {identity: self.runtime.state(identity) for identity in identities}
                safe = True
                for identity, state in states.items():
                    if identity in pending_acknowledgements and not state.get("navigation_active"):
                        if state.get("landing_acknowledged") is not True:
                            raise RuntimeError(
                                f"{identity}: navigation landing acknowledgement failed"
                            )
                    age = state.get("heartbeat_age_seconds")
                    safe = safe and (
                        state.get("connected") is True
                        and state.get("armed") is False
                        and state.get("navigation_active") is False
                        and type(age) in {int, float}
                        and 0 <= age <= 1.5
                        and bool(state.get("run_id"))
                    )
                if safe:
                    stable_since = stable_since or time.monotonic()
                    if time.monotonic() - stable_since >= 0.5:
                        previous_generations = {
                            key: state["run_id"] for key, state in states.items()
                        }
                        break
                else:
                    stable_since = None
                if time.monotonic() >= deadline:
                    raise TimeoutError(
                        "Master Reset could not establish fresh disarmed fleet quorum"
                    )
                time.sleep(0.05)
            self._phase("finalizing_capture", previous_generations=previous_generations)
            if self._cancel.is_set():
                raise InterruptedError(
                    "Restore cancelled after safe landing; no services restarted"
                )
            if self.replay is not None:
                self.replay.close(reason="master_reset")
            if self._cancel.is_set():
                raise InterruptedError(
                    "Restore cancelled after capture finalization; no services restarted"
                )
            # Owning adapter rechecks safety immediately before restarting each service.
            self._phase("restarting_services")
            result = self.runtime.restart(identities)
            for identity in identities:
                state = self.runtime.state(identity)
                if (
                    state.get("connected") is not True
                    or state.get("ready") is not True
                    or state.get("armed") is not False
                    or state.get("run_id") == previous_generations[identity]
                ):
                    raise RuntimeError(f"{identity}: fresh ready/disarmed controller not verified")
            if self._cancel.is_set():
                raise InterruptedError("Restore cancelled after safe restart verification")
            self._phase("restoring_defaults", restart=result)
            defaults = self.runtime.defaults()
            self._phase(
                "completed", active=False, defaults=defaults, finished_wall_time=time.time()
            )
        except (OSError, ValueError, RuntimeError, KeyError, TypeError) as error:
            self._phase(
                "cancelled" if isinstance(error, InterruptedError) else "error",
                active=False,
                error=str(error),
                finished_wall_time=time.time(),
            )
        finally:
            self.runtime.leave()
