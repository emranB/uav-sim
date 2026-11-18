"""Qualify the direct stereo pose path against one disarmed, isolated SITL vehicle."""

import argparse
import json
import os
import signal
import socket
import statistics
import tempfile
import threading
import time
from pathlib import Path

from gz.msgs10.clock_pb2 import Clock
from gz.transport13 import Node

from common.configuration.cameras import StereoConfiguration
from common.configuration.fleet import load_fleet
from common.configuration.visual_navigation import VisualEndpoints
from common.logging.async_recording import AsyncRecording
from common.logging.stereo import StereoFrame, StereoRecording
from common.logging.storage import write_json_atomic
from gazebo.visual_stereo import LatestStereoPair
from integrations.mavlink.visual_odometry import LocalVisualFrame, VisualOdometryLink
from integrations.px4.visual_aiding import (
    receive_status,
    send_status,
)
from integrations.vision.mapping import camera_configuration
from integrations.vision.native_pipe import NativePosePipe


def main() -> int:
    """Measure startup, cadence and wall age on an isolated live model."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vehicle", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=int, default=12, choices=range(5, 31), metavar="5..30")
    parser.add_argument("--features", type=int, default=1200, metavar="500..5000")
    parser.add_argument("--pyramid-levels", type=int, default=8, choices=range(8, 17))
    parser.add_argument("--record-poses", action="store_true")
    parser.add_argument(
        "--publisher-timeout-seconds", type=int, default=420, choices=range(60, 1201)
    )
    parser.add_argument("--native-executable", default="orb-recording")
    publication = parser.add_mutually_exclusive_group()
    publication.add_argument("--publish-disarmed", action="store_true")
    publication.add_argument("--flight-publisher", action="store_true")
    publication.add_argument("--return-workload", type=Path)
    args = parser.parse_args()
    if os.environ.get("GZ_PARTITION") != "uav-visual-odometry":
        parser.error("Use the isolated uav-visual-odometry Gazebo partition")
    fleet = tuple(vehicle for vehicle in load_fleet() if vehicle.enabled)
    selected = [vehicle for vehicle in fleet if vehicle.vehicle_id == args.vehicle]
    if len(selected) != 1:
        parser.error("Choose one explicitly configured, enabled visual vehicle")
    vehicle = selected[0]
    endpoints = VisualEndpoints.for_vehicle(vehicle)
    workload = (
        json.loads(args.return_workload.read_text()) if args.return_workload is not None else None
    )
    if workload is not None and workload.get("rig") != "visual_return_rig":
        parser.error("The timeline must identify the scripted return camera fixture")
    settings = StereoConfiguration.configured(maximum_rate_hz=50)
    if args.output.exists():
        parser.error("Use a new evidence directory")
    recording = StereoRecording(
        args.output,
        (StereoFrame(0, "", "", settings.width, settings.height),),
        {
            "baseline_m": settings.baseline_m,
            "horizontal_fov_rad": settings.horizontal_fov_rad,
            "distortion": "none",
        },
    )
    calibration = (
        camera_configuration(
            recording,
            settings.rate_hz,
            feature_count=args.features,
            pyramid_levels=args.pyramid_levels,
        )
        + "loopClosing: 0\n"
    )
    node = Node()
    simulation_now_ns = -1
    link = None
    clock_link_error = None

    def receive_clock(message: Clock) -> None:
        nonlocal simulation_now_ns, clock_link_error
        simulation_now_ns = message.sim.sec * 1_000_000_000 + message.sim.nsec
        active_link = link
        if active_link is not None and clock_link_error is None:
            try:
                active_link.poll(simulation_now_ns)
            except OSError as error:
                clock_link_error = error

    if not node.subscribe(Clock, "/world/default/clock", receive_clock):
        raise RuntimeError("Cannot subscribe to the isolated simulation clock")
    source = LatestStereoPair(
        node,
        vehicle,
        settings,
        model_name=workload["rig"] if workload is not None else None,
    )
    samples = []
    issues = []
    started = time.monotonic()
    run_marker = vehicle.runtime_directory / "px4.run"
    previous_timestamp_ns = -1
    stopping = threading.Event()
    signal.signal(signal.SIGINT, lambda signum, frame: stopping.set())
    signal.signal(signal.SIGTERM, lambda signum, frame: stopping.set())
    visual_frame = None
    visual_generation = None
    published = 0
    controller_generation = ""
    stale_poses = 0
    recorder = AsyncRecording(args.output / "poses.jsonl") if args.record_poses else None
    recording_status = None
    status_channel = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    status_channel.setblocking(False)
    fault_channel = None
    configuration_directory = tempfile.TemporaryDirectory(prefix="uav-visual-pose-", dir="/dev/shm")
    try:
        configuration = Path(configuration_directory.name) / "camera.yaml"
        configuration.write_text(calibration, encoding="utf-8")
        controller_generation = run_marker.read_text().strip()
        if args.flight_publisher:
            fault_channel = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            fault_channel.bind(("127.0.0.1", endpoints.vision_fault_port))
            fault_channel.setblocking(False)
        if args.publish_disarmed or args.flight_publisher:
            link = VisualOdometryLink(
                vehicle.system_id,
                allow_armed=args.flight_publisher,
                controller_port=endpoints.controller_port,
                publisher_port=endpoints.publisher_port,
            )
        with NativePosePipe(
            configuration,
            args.output / "native.log",
            args.native_executable,
        ) as estimator:
            deadline = started + (
                args.publisher_timeout_seconds if args.flight_publisher else 310 if workload else 45
            )
            measured_until = None
            while not stopping.is_set() and time.monotonic() < deadline:
                fault = receive_status(fault_channel) if fault_channel is not None else None
                if run_marker.read_text().strip() != controller_generation:
                    raise RuntimeError("Controller generation changed; publication stopped")
                if fault and (
                    fault.get("controller_run_id") == controller_generation
                    and fault.get("vehicle_id", "vehicle_1") == vehicle.vehicle_id
                ):
                    raise RuntimeError("Injected visual tracking loss; publication stopped")
                pair = source.wait_after(previous_timestamp_ns, 3)
                previous_timestamp_ns = pair.timestamp_ns
                submitted = time.monotonic()
                estimate = estimator.estimate(
                    [pair.left, pair.right], pair.timestamp_ns, stopping, timeout_seconds=10
                )
                if estimate is None:
                    break
                received = time.monotonic()
                if clock_link_error is not None:
                    raise RuntimeError(f"MAVLink clock exchange failed: {clock_link_error}")
                samples.append(
                    {
                        "timestamp_ns": pair.timestamp_ns,
                        "pair_sequence": pair.sequence,
                        "seconds_since_start": received - started,
                        "image_receipt_age_wall_seconds": received - pair.received_monotonic,
                        "processing_seconds": received - submitted,
                        "position_valid": estimate["position_valid"],
                        "map_generation": estimate.get("map_generation"),
                        "position_map_m": estimate["position_map_m"],
                        "orientation_xyzw": estimate["orientation_xyzw"],
                    }
                )
                if recorder is not None:
                    recorder.offer(dict(samples[-1]))
                if link is not None:
                    if visual_frame is not None and (
                        not estimate["position_valid"]
                        or estimate.get("map_generation") != visual_generation
                    ):
                        raise RuntimeError(
                            "Visual tracking or map origin changed; publication stopped"
                        )
                    if (
                        visual_frame is None
                        and estimate["position_valid"]
                        and estimate.get("map_generation") is not None
                        and link.disarmed
                        and link.attitude is not None
                        and received - link.attitude_received <= 0.1
                        and received - pair.received_monotonic <= 0.25
                    ):
                        visual_frame = LocalVisualFrame(
                            settings,
                            estimate["position_map_m"],
                            estimate["orientation_xyzw"],
                            link.attitude,
                        )
                        visual_generation = estimate["map_generation"]
                    if visual_frame is not None:
                        if received - pair.received_monotonic > 0.25:
                            # Never feed an expired estimate. The control guard
                            # times out against the last published capture.
                            stale_poses += 1
                            continue
                        position, orientation = visual_frame.to_ned(
                            estimate["position_map_m"], estimate["orientation_xyzw"]
                        )
                        link.publish(pair.timestamp_ns, simulation_now_ns, position, orientation)
                        published += 1
                        samples[-1]["published_count"] = published
                        if args.flight_publisher:
                            send_status(
                                status_channel,
                                endpoints.status_port,
                                {
                                    "kind": "pose",
                                    "vehicle_id": vehicle.vehicle_id,
                                    "controller_run_id": controller_generation,
                                    "monotonic": pair.received_monotonic,
                                    "healthy": True,
                                    "map_generation": visual_generation,
                                    "timestamp_ns": pair.timestamp_ns,
                                    "published_count": published,
                                    "receipt_age_wall_seconds": received - pair.received_monotonic,
                                    "horizontal_ned_m": [float(position[0]), float(position[1])],
                                },
                            )
                if measured_until is None and len(samples) >= 2 and estimate["position_valid"]:
                    measured_until = received + args.seconds
                if (
                    not args.flight_publisher
                    and workload is None
                    and measured_until is not None
                    and received >= measured_until
                ):
                    break
                if workload is not None and pair.timestamp_ns >= workload["return_end_ns"]:
                    break
    except (OSError, RuntimeError, TimeoutError, ValueError) as error:
        issues.append(str(error))
    finally:
        node.unsubscribe("/world/default/clock")
        if link is not None:
            link.close()
        source.close()
        if args.flight_publisher:
            send_status(
                status_channel,
                endpoints.status_port,
                {
                    "kind": "pose",
                    "vehicle_id": vehicle.vehicle_id,
                    "controller_run_id": controller_generation,
                    "monotonic": time.monotonic(),
                    "healthy": False,
                },
            )
        if fault_channel is not None:
            fault_channel.close()
        status_channel.close()
        if recorder is not None:
            recording_status = recorder.close()
        configuration_directory.cleanup()

    first_tracking_index = next(
        (index for index, sample in enumerate(samples) if index and sample["position_valid"]),
        len(samples),
    )
    warmup = (
        [sample for sample in samples if sample["timestamp_ns"] < workload["return_start_ns"]]
        if workload is not None
        else []
    )
    measured = (
        [sample for sample in samples if workload["return_start_ns"] <= sample["timestamp_ns"]]
        if workload is not None
        else samples[first_tracking_index:]
    )
    if (args.publish_disarmed or args.flight_publisher) and not published:
        issues.append("No visual odometry was published")
    rate_hz = 0.0
    ages = [sample["image_receipt_age_wall_seconds"] for sample in measured]
    if len(measured) < 2:
        issues.append("No sustained visual initialization")
    else:
        duration = (measured[-1]["timestamp_ns"] - measured[0]["timestamp_ns"]) / 1e9
        rate_hz = (len(measured) - 1) / duration
        required_seconds = args.seconds if workload is None else 0
        if (
            measured[-1]["seconds_since_start"] - measured[0]["seconds_since_start"]
            < required_seconds
        ):
            issues.append("Sustained measurement window did not finish")
        if workload is not None and measured[-1]["timestamp_ns"] < workload["return_end_ns"]:
            issues.append("Return workload did not finish")
        if rate_hz < 30:
            issues.append("Visual stream below 30 distinct poses per simulation second")
        if max(ages) > 0.25:
            issues.append("Visual stream exceeded the 250 ms image receipt age budget")
        if (
            not all(sample["position_valid"] for sample in measured)
            or len({sample["map_generation"] for sample in measured}) != 1
            or measured[0]["map_generation"] is None
        ):
            issues.append("Tracking or coordinate generation changed")

    output = args.output / "report.json"
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "camera.yaml").write_text(calibration, encoding="utf-8")
    write_json_atomic(
        output,
        {
            "schema_version": 1,
            "scope": "Isolated live stereo position publisher"
            if args.flight_publisher
            else "Isolated live stereo path; disarmed only",
            "vehicle_id": vehicle.vehicle_id,
            "controller_run_id": controller_generation,
            "requested_camera_rate_hz": settings.rate_hz,
            "features": args.features,
            "passed": not issues,
            "issues": issues,
            "pairs_received": source.completed,
            "odometry_messages_published": published,
            "timesync_replies": link.timesync_replies if link is not None else 0,
            "stale_poses_discarded": stale_poses,
            "optional_recording": recording_status,
            "transport": "native-pose-pipe",
            "source_rate_sim_hz": (
                (measured[-1]["pair_sequence"] - measured[0]["pair_sequence"])
                / ((measured[-1]["timestamp_ns"] - measured[0]["timestamp_ns"]) / 1e9)
                if len(measured) > 1
                else None
            ),
            "pairs_skipped_before_estimation": (
                measured[-1]["pair_sequence"] - measured[0]["pair_sequence"] - len(measured) + 1
                if len(measured) > 1
                else None
            ),
            "return_workload": workload,
            "workload_warmup_untracked_frames": sum(
                not sample["position_valid"] for sample in warmup
            ),
            "workload_warmup_map_generations": sorted(
                {
                    sample["map_generation"]
                    for sample in warmup
                    if sample["map_generation"] is not None
                }
            ),
            "distinct_pose_rate_sim_hz": rate_hz,
            "median_image_receipt_age_wall_seconds": statistics.median(ages) if ages else None,
            "max_image_receipt_age_wall_seconds": max(ages, default=None),
            "samples": samples,
        },
    )
    print(output)
    return int(bool(issues))


if __name__ == "__main__":
    raise SystemExit(main())
