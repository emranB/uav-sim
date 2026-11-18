"""Simultaneous estimated paths and measured aiding state for visual comparisons."""

import time
from collections import deque
from typing import TYPE_CHECKING

from PySide6.QtCore import QPointF, Qt, QTimer
from PySide6.QtGui import QColor, QPainter, QPaintEvent, QPen
from PySide6.QtWidgets import QLabel, QVBoxLayout, QWidget

from control_panel.health import VehicleHealthBadges

if TYPE_CHECKING:
    from desktop.client import SimulationClient


class FleetPathView(QWidget):
    """Bounded estimated stereo paths; configured launch offsets are display bindings only."""

    def __init__(self) -> None:
        super().__init__()
        self.setMinimumHeight(180)
        self.paths: dict[str, deque] = {}
        self.rows: dict[str, dict] = {}
        self.generations: dict[str, tuple] = {}

    def receive(self, rows: list[dict]) -> None:
        self.rows = {row["vehicle_id"]: row for row in rows}
        for identity in set(self.paths) - self.rows.keys():
            self.paths.pop(identity)
            self.generations.pop(identity, None)
        for identity, row in self.rows.items():
            nav = row["navigation"]
            source = nav.get("visual_localization") or {}
            pose, aiding = source.get("pose", {}), source.get("aiding", {})
            generation = pose.get("controller_run_id"), pose.get("map_generation")
            if generation != self.generations.get(identity):
                self.paths[identity] = deque(maxlen=2000)
                self.generations[identity] = generation
            now = time.monotonic()
            if (
                pose.get("healthy")
                and "horizontal_ned_m" in pose
                and 0 <= now - pose.get("monotonic", 0) <= 0.25
                and 0 <= now - aiding.get("monotonic", 0) <= 0.5
            ):
                north, east = pose["horizontal_ned_m"]
                launch = row["display_launch_enu_m"]
                point = (east + launch[0], north + launch[1])
                path = self.paths[identity]
                if not path or point != path[-1]:
                    path.append(point)
        self.update()

    def paintEvent(self, event: QPaintEvent) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor("#0d1725"))
        scale = min(self.width() / 20, self.height() / 16)

        def pixel(point: tuple) -> QPointF:
            return QPointF(
                self.width() / 2 + (point[0] - 3) * scale, self.height() * 0.72 - point[1] * scale
            )

        for index, (identity, row) in enumerate(self.rows.items()):
            colour = QColor(("#69d9bc", "#73b9ff", "#e3b773")[index % 3])
            painter.setPen(QPen(colour, 2))
            path = list(self.paths.get(identity, []))
            for start, end in zip(path, path[1:]):
                painter.drawLine(pixel(start), pixel(end))
            home = tuple(row["display_launch_enu_m"][:2])
            painter.drawEllipse(pixel(home), 5, 5)
            painter.drawText(pixel(home) + QPointF(8, 16), row["label"] + " home")
            mission = row["navigation"].get("saved_mission") or {}
            if mission:
                offset = mission["goal_offset_enu_m"]
                waypoint = home[0] + offset[0], home[1] + offset[1]
                painter.setPen(QPen(colour, 1, Qt.PenStyle.DashLine))
                painter.drawLine(pixel(home), pixel(waypoint))
                painter.drawText(pixel(waypoint) + QPointF(5, -5), "WP1")
            if path:
                painter.setPen(QPen(colour, 6))
                painter.drawPoint(pixel(path[-1]))
        painter.setPen(QColor("#91a8be"))
        painter.drawText(8, 18, "Camera-estimated paths · metres · launch offsets for display")


class FleetComparisonPanel(QWidget):
    """Show each vehicle's requested GPS state separately from observed native fusion."""

    def __init__(self, client: "SimulationClient") -> None:
        super().__init__()
        self.last_payload: dict | None = None
        self.last_received_monotonic = 0.0
        layout = QVBoxLayout(self)
        self.heading = QLabel("Independent routes · camera-estimated paths · shared scene")
        self.heading.setWordWrap(True)
        layout.addWidget(self.heading)
        self.paths = FleetPathView()
        layout.addWidget(self.paths, 1)
        self.health = VehicleHealthBadges(columns=4)
        layout.addWidget(self.health)
        self.sources = QLabel("Waiting for both vehicle sources")
        self.sources.setWordWrap(True)
        self.sources.setObjectName("operationFeedback")
        layout.addWidget(self.sources)
        self.timer = QTimer(self)
        self.timer.setInterval(250)

        def refresh() -> None:
            client.fetch("navigation/fleet")
            if self.last_payload is not None:
                self.render(self.last_payload)

        self.timer.timeout.connect(refresh)
        self.timer.start()
        client.resource_received.connect(self.receive)

    def receive(self, resource: str, payload: dict) -> None:
        if resource != "navigation/fleet":
            return
        self.last_payload = payload
        self.last_received_monotonic = time.monotonic()
        self.render(payload)

    def render(self, payload: dict) -> None:
        """Keep API snapshot age distinct from the source's measured receipt age."""
        rows = payload["items"]
        self.heading.setText(
            f"Scene: {payload.get('scene_id', 'unknown')} · independent routes · estimated paths"
        )
        self.paths.receive(rows)
        summaries = []
        display_age = time.monotonic() - self.last_received_monotonic
        self.health.present(rows, display_age)
        for row in rows:
            nav = row["navigation"]
            source = nav.get("visual_localization") or {}
            aiding, pose = source.get("aiding", {}), source.get("pose", {})
            now = self.last_received_monotonic
            pose_age = now - pose.get("monotonic", 0)
            aiding_age = now - aiding.get("monotonic", 0)
            fresh = display_age <= 1.5 and 0 <= pose_age <= 0.25 and 0 <= aiding_age <= 0.5
            requested = aiding.get("gps_requested_enabled")
            gps_request = "on" if requested is True else "off" if requested is False else "unknown"
            flags = (
                (
                    f"GNSS xy/vel/h: {int(bool(aiding.get('gnss_xy_active')))}/"
                    f"{int(bool(aiding.get('gnss_velocity_active')))}/"
                    f"{int(bool(aiding.get('gnss_height_active')))} · "
                    f"vision xy/h: {int(bool(aiding.get('vision_xy_active')))}/"
                    f"{int(bool(aiding.get('vision_height_active')))} · "
                    f"baro h: {int(bool(aiding.get('barometric_height_active')))}"
                )
                if aiding
                else "Native fusion unavailable"
            )
            fusion = (
                "GNSS + VISION"
                if aiding.get("gnss_xy_active") and aiding.get("vision_xy_active")
                else "VISION + IMU"
                if aiding.get("vision_xy_active")
                else "GNSS"
                if aiding.get("gnss_xy_active")
                else "NO HORIZONTAL AIDING"
            )
            if requested is False and any(
                aiding.get(key)
                for key in (
                    "gnss_xy_active",
                    "gnss_velocity_active",
                    "gnss_height_active",
                )
            ):
                fusion = "GPS LOSS TRANSITION · " + fusion
            mission = nav.get("saved_mission") or {}
            summaries.append(
                f"{row['label']} · {'ARMED' if row['armed'] else 'DISARMED'} · "
                f"{nav.get('state', 'unavailable')} / {nav.get('current_waypoint') or nav.get('leg') or 'idle'}\n"
                f"GPS requested {gps_request} · fix {aiding.get('gps_fix_type', '—')} · "
                f"satellites {aiding.get('satellites_used', '—')} · {fusion}\n{flags}\n"
                + (
                    f"Source age at snapshot: pose {pose_age:.2f}s · aiding {aiding_age:.2f}s"
                    if fresh
                    else "STALE / FROZEN"
                )
                + f" · mission {mission.get('name', 'none')} · map {pose.get('map_generation', '—')}"
            )
        self.sources.setText("\n\n".join(summaries))
