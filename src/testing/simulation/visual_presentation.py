"""Bounded native desktop fixture for presentation captures and real widget commands."""

import argparse
import json
import os
import re
import signal
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from PySide6.QtCore import QObject, QPoint, QTimer, Signal, Slot
from PySide6.QtWidgets import QApplication, QWidget

from common.logging.storage import ensure_capacity, write_json_atomic
from desktop.flight_actions import FlightActionRow
from desktop.theme import apply_theme
from desktop.viewer import ViewerPanel
from desktop.window import MainWindow


class PresentationHarness(QObject):
    """Test-only requests operate the existing widgets, never a controller transport."""

    request_received = Signal(dict)

    def __init__(self, window: MainWindow, root: Path) -> None:
        super().__init__(window)
        self.window, self.root = window, root
        self.row = window.findChild(FlightActionRow)
        self.case = "startup"
        self.captured: set[tuple[str, str]] = set()
        self.pending: dict[str, str] = {}
        self.futures = []
        self.writer = ThreadPoolExecutor(max_workers=1, thread_name_prefix="presentation-writer")
        self.stopping = threading.Event()
        self.reader = threading.Thread(target=self.read_requests, daemon=True)
        self.request_received.connect(self.apply_request)
        window.client.resource_received.connect(self.receive)
        window.client.target_operation_result.connect(self.acknowledge)
        self.reader.start()

    def read_requests(self) -> None:
        seen = set()
        while not self.stopping.wait(0.2) and len(seen) < 64:
            for filename in ("presentation-context.json", "quick-action-request.json"):
                try:
                    encoded = (self.root / filename).read_bytes()
                    if len(encoded) > 65536:
                        raise ValueError("Presentation request exceeds its 64 KiB budget")
                    request = json.loads(encoded)
                except FileNotFoundError:
                    continue
                except (OSError, ValueError) as error:
                    print(f"Presentation request unavailable: {error}", flush=True)
                    continue
                identifier = request.get("request_id", "")
                if not isinstance(identifier, str) or not re.fullmatch(
                    r"[a-z0-9-]{1,64}", identifier
                ):
                    continue
                if identifier not in seen:
                    seen.add(identifier)
                    self.request_received.emit(request)

    @Slot(dict)
    def apply_request(self, request: dict) -> None:
        identifier = request["request_id"]
        if request.get("kind") == "context":
            case = request.get("case", "")
            if not isinstance(case, str) or not re.fullmatch(r"[a-z0-9-]{1,64}", case):
                return
            self.case = case
            self.row.clear_selection()
            return
        action = request.get("action")
        targets = request.get("target_ids")
        if (
            action not in {"start", "land"}
            or not isinstance(targets, list)
            or any(target not in self.row.vehicles for target in targets)
        ):
            self.save_ack(identifier, {"accepted": False, "error": "Invalid widget request"})
            return
        self.row.clear_selection()
        for target in targets:
            self.row.checkboxes[target].setChecked(True)
        if not self.row.buttons[action].isEnabled():
            self.save_ack(
                identifier, {"accepted": False, "error": self.row.buttons[action].toolTip()}
            )
            return
        self.row.buttons[action].click()
        self.pending[self.row.pending_id] = identifier
        if request.get("change_selection_after_dispatch"):
            self.row.clear_selection()
            for target in request["change_selection_after_dispatch"]:
                self.row.checkboxes[target].setChecked(True)

    @Slot(str, dict)
    def acknowledge(self, request_id: str, result: dict) -> None:
        identifier = self.pending.pop(request_id, None)
        if identifier is not None:
            self.save_ack(
                identifier,
                {
                    **result,
                    "accepted": all(row.get("accepted") for row in result["results"].values()),
                    "visible_selection_after_dispatch": sorted(self.row.selected_ids),
                    "feedback": self.row.feedback.text(),
                },
            )

    def save_ack(self, identifier: str, result: dict) -> None:
        self.futures.append(
            self.writer.submit(
                write_json_atomic,
                self.root / f"{identifier}-ack.json",
                result,
            )
        )

    @Slot(str, dict)
    def receive(self, resource: str, payload: dict) -> None:
        if resource != "navigation/fleet" or len(payload["items"]) != 2:
            return
        rows = payload["items"]
        sources = [row["navigation"].get("visual_localization") or {} for row in rows]
        if (
            all(not row["armed"] and row["navigation"].get("state") == "idle" for row in rows)
            and all(
                source.get("pose", {}).get("healthy")
                and source.get("aiding", {}).get("source_samples_fresh")
                and source.get("aiding", {}).get("vision_xy_active")
                and time.monotonic() - source.get("aiding", {}).get("monotonic", 0) <= 0.5
                for source in sources
            )
            and all(
                panel.foreign_window is not None for panel in self.window.findChildren(ViewerPanel)
            )
        ):
            self.capture("preflight", rows)
        armed = all(row["armed"] for row in rows)
        if (
            armed
            and all(row["navigation"].get("state") == "navigating" for row in rows)
            and all(
                sum(value * value for value in source.get("pose", {}).get("horizontal_ned_m", []))
                >= 4
                for source in sources
            )
        ):
            self.capture("outbound", rows)
        aiding = [
            (row["navigation"].get("visual_localization") or {}).get("aiding", {}) for row in rows
        ]
        if armed and any(
            state.get("gps_loss_injected") and state.get("gnss_xy_active") for state in aiding
        ):
            self.capture("gps-transition", rows)
        if (
            armed
            and any(state.get("gps_loss_injected") for state in aiding)
            and all(
                not state.get("gps_loss_injected")
                or (not state.get("gnss_xy_active") and state.get("vision_xy_active"))
                for state in aiding
            )
        ):
            self.capture("camera-after-gps-loss", rows)
        if all(not row["armed"] and row["navigation"].get("state") == "reached" for row in rows):
            self.capture("landed", rows)
        pending = []
        for future in self.futures:
            if future.done():
                future.result()
            else:
                pending.append(future)
        self.futures = pending

    def capture(self, phase: str, rows: list[dict]) -> None:
        key = self.case, phase
        if key in self.captured or len(self.captured) >= 32:
            return
        self.captured.add(key)
        snapshot = json.loads(json.dumps(rows))
        # Resource delivery schedules widget paints. Capture after that event
        # cycle so the native image shows the recorded source state.
        QTimer.singleShot(100, lambda: self.capture_rendered(key, snapshot))

    def capture_rendered(self, key: tuple[str, str], snapshot: list[dict]) -> None:
        image = self.window.screen().grabWindow(int(self.window.winId())).toImage()
        qgc = self.window.findChild(QWidget, "qgcEmbeddedView")
        geometry = {
            "qgc_bottom_y": qgc.mapTo(self.window, QPoint(0, qgc.height())).y(),
            "quick_row_top_y": self.row.mapTo(self.window, QPoint(0, 0)).y(),
            "selected_ids": sorted(self.row.selected_ids),
            "native_viewers": {
                panel.window_file.stem: panel.foreign_window is not None
                for panel in self.window.findChildren(ViewerPanel)
            },
        }

        def write() -> None:
            ensure_capacity(self.root, image.sizeInBytes())
            destination = self.root / f"{key[0]}-{key[1]}.png"
            destination.parent.mkdir(parents=True, exist_ok=True)
            if not image.save(str(destination)):
                raise OSError(f"Cannot save presentation capture: {destination}")
            write_json_atomic(
                destination.with_suffix(".json"),
                {
                    "case": key[0],
                    "phase": key[1],
                    "wall_time": time.time(),
                    "geometry": geometry,
                    "vehicles": snapshot,
                },
            )

        self.futures.append(self.writer.submit(write))

    def close(self) -> None:
        self.stopping.set()
        self.reader.join(timeout=2)
        self.writer.shutdown(wait=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence", type=Path, required=True)
    arguments = parser.parse_args()
    if os.environ.get("GZ_PARTITION") != "uav-visual-odometry":
        parser.error("Use the isolated visual comparison deployment")
    arguments.evidence.mkdir(parents=True, exist_ok=True)
    application = QApplication([])
    apply_theme(application)
    window = MainWindow()
    harness = PresentationHarness(window, arguments.evidence)
    signal.signal(signal.SIGTERM, lambda signum, frame: window.close())
    signal.signal(signal.SIGINT, lambda signum, frame: window.close())
    window.showMaximized()
    marker = Path("/runtime/Desktop.window")
    marker.write_text(str(int(window.winId())))
    QTimer.singleShot(1_800_000, window.close)
    try:
        return application.exec()
    finally:
        harness.close()
        marker.unlink(missing_ok=True)


if __name__ == "__main__":
    raise SystemExit(main())
