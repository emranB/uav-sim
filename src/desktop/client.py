"""Asynchronous desktop API client; no blocking network calls on the UI thread."""

import json
import logging
import os
from collections.abc import Callable
from pathlib import Path

from PySide6.QtCore import QObject, QTimer, QUrl, Signal
from PySide6.QtNetwork import QNetworkAccessManager, QNetworkReply, QNetworkRequest

from common.configuration.preferences import DesktopPreferences
from common.logging.structured import recording_handler


class SimulationClient(QObject):
    telemetry_received = Signal(dict)
    error_received = Signal(str)
    operation_finished = Signal(str)
    mission_received = Signal(dict)
    workspace_reset = Signal()
    resource_received = Signal(str, dict)
    operation_result = Signal(str, dict)
    operation_failed = Signal(str, str)
    preferences_changed = Signal(dict)
    selection_changed = Signal(str)
    camera_image_received = Signal(str, bytes)
    camera_image_failed = Signal(str, str)
    target_operation_result = Signal(str, dict)
    replay_operation_result = Signal(str, dict)
    gnss_operation_result = Signal(str, dict)
    restore_operation_result = Signal(str, dict)
    restore_completed = Signal(dict)

    def __init__(self, url: str = "http://127.0.0.1:8080") -> None:
        super().__init__()
        self.url = url
        self.vehicle_id = "vehicle_1"
        self.target_ids = [self.vehicle_id]
        self.fleet_ids = [self.vehicle_id]
        self.apply_to_all = False
        self.explicit_targets = False
        self.network = QNetworkAccessManager(self)
        self.pending = False
        self._closing = False
        self.last_epoch = None
        self.resources_pending: set[str] = set()
        self.images_pending: set[tuple[str, str]] = set()
        self.poll_count = 0
        self.preference_store = DesktopPreferences(
            Path(os.environ.get("OUTPUT_DIR", "outputs")) / "preferences/desktop.json"
        )
        self.preferences = self.preference_store.load()
        diagnostics = logging.getLogger("workspace.desktop")
        if not diagnostics.handlers:
            diagnostics.addHandler(
                recording_handler(self.preference_store.path.parents[1] / "logs", "desktop")
            )
        self.error_received.connect(
            lambda message: diagnostics.warning(message, extra={"vehicle_id": self.vehicle_id})
        )
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(self.preferences["poll_interval_ms"])

    def save_preferences(self, values: dict) -> None:
        self.preferences = self.preference_store.save(values)
        self.timer.setInterval(self.preferences["poll_interval_ms"])
        self.preferences_changed.emit(self.preferences)

    def close(self) -> None:
        """Cancel outstanding network work before widgets and the Qt loop are released."""
        self._closing = True
        self.timer.stop()
        for reply in self.network.findChildren(QNetworkReply):
            reply.blockSignals(True)
            reply.abort()
            reply.deleteLater()

    def select_vehicle(self, vehicle_id: str, apply_to_all: bool = False) -> None:
        if vehicle_id not in self.fleet_ids:
            raise ValueError("Choose a configured vehicle")
        self.vehicle_id = vehicle_id
        self.apply_to_all = apply_to_all
        if not self.explicit_targets:
            self.target_ids = list(self.fleet_ids) if apply_to_all else [vehicle_id]
        self.selection_changed.emit(vehicle_id)
        self.telemetry_received.emit({"connected": False, "vehicle_id": vehicle_id})
        for resource in ("workspace", "missions", "experiments", "recordings"):
            self.fetch(resource)

    def set_action_targets(self, vehicle_ids: tuple[str, ...]) -> None:
        """Use the workspace checkboxes for mutations; inspection never changes this scope."""
        if len(set(vehicle_ids)) != len(vehicle_ids) or any(
            identity not in self.fleet_ids for identity in vehicle_ids
        ):
            raise ValueError("Choose unique configured vehicle targets")
        self.explicit_targets = True
        self.target_ids = list(vehicle_ids)

    def resource_url(self, resource: str, vehicle_id: str) -> QUrl:
        separator = "&" if "?" in resource else "?"
        return QUrl(f"{self.url}/{resource}{separator}vehicle_id={vehicle_id}")

    def fetch_camera_image(self, side: str) -> None:
        """Bound preview requests and discard images from a previous vehicle focus."""
        if side not in {"left", "right"}:
            raise ValueError("Camera side must be left or right")
        vehicle_id = self.vehicle_id
        key = vehicle_id, side
        if self._closing or key in self.images_pending:
            return
        self.images_pending.add(key)
        request = QNetworkRequest(self.resource_url(f"perception/image?side={side}", vehicle_id))
        request.setTransferTimeout(2000)
        reply = self.network.get(request)

        def finished() -> None:
            self.images_pending.discard(key)
            try:
                if vehicle_id != self.vehicle_id or self._closing:
                    return
                contents = bytes(reply.readAll())
                if reply.error() != QNetworkReply.NetworkError.NoError:
                    self.camera_image_failed.emit(side, "Camera image is unavailable or stale")
                elif len(contents) > 1024 * 1024:
                    self.camera_image_failed.emit(side, "Camera preview exceeds 1 MiB")
                else:
                    self.camera_image_received.emit(side, contents)
            finally:
                reply.deleteLater()

        reply.finished.connect(finished)

    def refresh(self) -> None:
        if self.pending or self._closing:
            return
        self.pending = True
        self.poll_count += 1
        if self.poll_count % 4 == 1:
            self.fetch("workspace")
            self.fetch("vehicles")
            self.fetch("perception")
        vehicle_id = self.vehicle_id
        request = QNetworkRequest(self.resource_url("telemetry", vehicle_id))
        request.setTransferTimeout(2000)
        reply = self.network.get(request)

        def finished() -> None:
            self.pending = False
            try:
                if vehicle_id != self.vehicle_id:
                    return
                payload = json.loads(bytes(reply.readAll()))
                epoch = payload.get("scene_epoch")
                if self.last_epoch is not None and epoch is not None and epoch != self.last_epoch:
                    self.workspace_reset.emit()
                if epoch is not None:
                    self.last_epoch = epoch
                self.telemetry_received.emit(payload)
            except (ValueError, TypeError):
                self.telemetry_received.emit({"connected": False})
                self.error_received.emit(reply.errorString())
            finally:
                reply.deleteLater()

        reply.finished.connect(finished)

    def execute(self, operation: str, body: dict | None = None) -> None:
        if self._closing:
            return
        body = dict(body or {})
        grouped = (
            operation
            in {
                "start",
                "stop",
                "reset",
                "vehicles/configure",
                "missions/load",
                "experiments/load",
            }
            or operation.startswith(("sensors/", "interference/"))
            or (
                self.explicit_targets
                and (operation == "mission" or operation.startswith("navigation/"))
            )
        )
        if grouped and not self.target_ids:
            reason = "Choose targets below the mission map before applying changes"
            self.operation_failed.emit(operation, reason)
            self.error_received.emit(reason)
            return
        body["target_ids"] = list(self.target_ids) if grouped else [self.vehicle_id]
        vehicle_id = self.vehicle_id
        request = QNetworkRequest(QUrl(self.url + "/" + operation))
        request.setRawHeader(b"X-Request-Source", b"desktop")
        request.setHeader(QNetworkRequest.KnownHeaders.ContentTypeHeader, "application/json")
        request.setTransferTimeout(80000 if len(body["target_ids"]) > 1 else 35000)
        reply = self.network.post(request, json.dumps(body).encode())

        def finished() -> None:
            try:
                payload = json.loads(bytes(reply.readAll()))
                if vehicle_id != self.vehicle_id:
                    return
                if "error" in payload:
                    self.operation_failed.emit(operation, payload["error"])
                    self.error_received.emit(payload["error"])
                else:
                    if "results" in payload:
                        payload = {**payload, **payload["results"].get(vehicle_id, {})}
                    self.operation_result.emit(operation, payload)
                    self.operation_finished.emit(operation)
                    self.fetch("workspace")
                    self.fetch("vehicles")
            except ValueError:
                self.operation_failed.emit(operation, reply.errorString())
                self.error_received.emit(reply.errorString())
            finally:
                reply.deleteLater()

        reply.finished.connect(finished)

    def execute_targets(
        self,
        operation: str,
        target_ids: list[str],
        request_id: str,
        *,
        stereo_horizontal_reference: bool = False,
    ) -> None:
        """Snapshot explicit shortcut targets and retain every result across focus changes."""
        targets = tuple(target_ids)
        if operation not in {"flight/start", "flight/land"}:
            raise ValueError("Unsupported flight shortcut")
        if (
            not targets
            or len(set(targets)) != len(targets)
            or any(target not in self.fleet_ids for target in targets)
        ):
            raise ValueError("Choose one or more configured vehicles explicitly")
        if self._closing:
            return
        request = QNetworkRequest(QUrl(self.url + "/" + operation))
        request.setRawHeader(b"X-Request-Source", b"desktop-quick-actions")
        request.setHeader(QNetworkRequest.KnownHeaders.ContentTypeHeader, "application/json")
        request.setTransferTimeout(80000 if len(targets) > 1 else 35000)
        body = {"target_ids": targets}
        if operation == "flight/start":
            body["stereo_horizontal_reference"] = stereo_horizontal_reference
        reply = self.network.post(request, json.dumps(body).encode())

        def finished() -> None:
            try:
                payload = json.loads(bytes(reply.readAll()))
                if not isinstance(payload, dict):
                    raise ValueError("Expected a command result object")
                results = payload.get("results")
                if not isinstance(results, dict):
                    reason = payload.get("error") or reply.errorString() or "No acknowledgement"
                    results = {target: {"accepted": False, "error": reason} for target in targets}
                self.target_operation_result.emit(
                    request_id,
                    {
                        "operation": operation,
                        "target_ids": list(targets),
                        "results": {
                            target: results.get(
                                target,
                                {
                                    "accepted": False,
                                    "error": "Vehicle acknowledgement missing",
                                },
                            )
                            for target in targets
                        },
                        "preflight_passed": payload.get("preflight_passed", False),
                    },
                )
                self.fetch("vehicles")
            except (ValueError, TypeError) as error:
                self.target_operation_result.emit(
                    request_id,
                    {
                        "operation": operation,
                        "target_ids": list(targets),
                        "results": {
                            target: {"accepted": False, "error": reply.errorString() or str(error)}
                            for target in targets
                        },
                    },
                )
            finally:
                reply.deleteLater()

        reply.finished.connect(finished)

    def execute_gnss_targets(
        self,
        target_ids: tuple[str, ...],
        enabled: bool,
        request_id: str,
    ) -> None:
        """Correlate a GNSS input acknowledgement with immutable targets across Inspect changes."""
        targets = tuple(target_ids)
        if (
            type(enabled) is not bool
            or not targets
            or len(set(targets)) != len(targets)
            or any(identity not in self.fleet_ids for identity in targets)
        ):
            raise ValueError("Choose unique configured GNSS targets and a true/false input")
        self._execute_global(
            "sensors/gnss",
            {"target_ids": list(targets), "enabled": enabled},
            request_id,
            self.gnss_operation_result.emit,
            "vehicles",
        )

    def execute_replay(self, operation: str, body: dict, request_id: str) -> None:
        """Keep replay scope/results independent of subsequent viewed-vehicle changes."""
        if operation not in {
            "replay/start",
            "replay/stop",
            "replay/keep",
            "replay/export",
            "replay/render",
            "replay/cancel",
        }:
            raise ValueError("Unsupported replay operation")
        self._execute_global(
            operation, body, request_id, self.replay_operation_result.emit, "replay"
        )

    def execute_restore(self, request_id: str, *, cancel: bool = False) -> None:
        """Request whole-UAV recovery independently of viewed or checkbox targets."""
        operation = "restore/cancel" if cancel else "restore/start"
        self._execute_global(
            operation, {}, request_id, self.restore_operation_result.emit, "restore"
        )

    def _execute_global(
        self,
        operation: str,
        body: dict,
        request_id: str,
        result: Callable[[str, dict], None],
        resource: str,
    ) -> None:
        if self._closing:
            return
        request = QNetworkRequest(QUrl(self.url + "/" + operation))
        request.setRawHeader(b"X-Request-Source", b"desktop-workspace")
        request.setHeader(QNetworkRequest.KnownHeaders.ContentTypeHeader, "application/json")
        request.setTransferTimeout(10000)
        reply = self.network.post(request, json.dumps(body).encode())

        def finished() -> None:
            try:
                payload = json.loads(bytes(reply.readAll()))
                if not isinstance(payload, dict):
                    raise ValueError("Expected a workspace operation result object")
                if reply.error() != QNetworkReply.NetworkError.NoError:
                    payload = {
                        "accepted": False,
                        "error": payload.get("error", reply.errorString()),
                    }
                result(request_id, payload)
                self.fetch(resource)
            except (ValueError, TypeError) as error:
                result(
                    request_id,
                    {
                        "accepted": False,
                        "error": reply.errorString() or str(error),
                    },
                )
            finally:
                reply.deleteLater()

        reply.finished.connect(finished)

    def fetch(self, resource: str) -> None:
        """Fetch a named panel resource without overlapping identical requests."""
        if self._closing:
            return
        vehicle_id = self.vehicle_id
        global_resource = resource in {"replay", "restore"}
        key = f"{'global' if global_resource else vehicle_id}:{resource}"
        if key in self.resources_pending:
            return
        self.resources_pending.add(key)
        url = (
            QUrl(self.url + "/" + resource)
            if global_resource
            else self.resource_url(resource, vehicle_id)
        )
        request = QNetworkRequest(url)
        request.setTransferTimeout(10000)
        reply = self.network.get(request)

        def finished() -> None:
            self.resources_pending.discard(key)
            try:
                if not global_resource and vehicle_id != self.vehicle_id:
                    return
                payload = json.loads(bytes(reply.readAll()))
                if "error" in payload:
                    self.error_received.emit(payload["error"])
                else:
                    if resource == "vehicles":
                        self.fleet_ids = [item["vehicle_id"] for item in payload["items"]]
                        if self.vehicle_id not in self.fleet_ids and self.fleet_ids:
                            self.select_vehicle(self.fleet_ids[0], self.apply_to_all)
                        if self.explicit_targets:
                            self.target_ids = [
                                key for key in self.target_ids if key in self.fleet_ids
                            ]
                        else:
                            self.target_ids = (
                                list(self.fleet_ids) if self.apply_to_all else [self.vehicle_id]
                            )
                    self.resource_received.emit(resource, payload)
            except (ValueError, TypeError):
                self.error_received.emit(reply.errorString())
            finally:
                reply.deleteLater()

        reply.finished.connect(finished)

    def download_mission(self) -> None:
        if self._closing:
            return
        vehicle_id = self.vehicle_id
        request = QNetworkRequest(self.resource_url("mission", vehicle_id))
        request.setTransferTimeout(35000)
        reply = self.network.get(request)

        def finished() -> None:
            try:
                if vehicle_id != self.vehicle_id:
                    return
                payload = json.loads(bytes(reply.readAll()))
                if "error" in payload:
                    self.error_received.emit(payload["error"])
                else:
                    self.mission_received.emit(payload)
            except ValueError:
                self.error_received.emit(reply.errorString())
            finally:
                reply.deleteLater()

        reply.finished.connect(finished)
