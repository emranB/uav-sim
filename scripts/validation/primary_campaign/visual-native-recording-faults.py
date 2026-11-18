"""Check native estimator receipts while the optional pose sink stalls or raises ENOSPC."""

import errno
import json
import threading
import time
from pathlib import Path
from unittest.mock import patch

from common.logging.async_recording import AsyncRecording
from common.logging.stereo import StereoRecording
from common.logging.storage import write_json_atomic
from integrations.vision.mapping import camera_configuration
from integrations.vision.native_pipe import NativePosePipe

root = Path("/workspace/outputs")
source = StereoRecording.open(root / "return-clip/report.json")
results = {}
for fault in ("slow", "enospc"):
    directory = root / "async-faults" / fault
    directory.mkdir(parents=True)
    path = directory / "poses.jsonl"
    calibration = directory / "camera.yaml"
    calibration.write_text(camera_configuration(source, 40, 1200) + "loopClosing: 0\n")
    release = threading.Event()
    entered = threading.Event()
    open_path = Path.open

    class FaultSink:
        def __enter__(self):
            return self

        def __exit__(self, *_):
            return None

        def write(self, payload):
            entered.set()
            if fault == "enospc":
                raise OSError(errno.ENOSPC, "Injected pose recording write failure")
            release.wait(30)
            return len(payload)

    def open_with_fault(self, *args, **kwargs):
        return FaultSink() if self == path else open_path(self, *args, **kwargs)

    stopping = threading.Event()
    received = []
    latencies = []
    with patch.object(Path, "open", open_with_fault):
        recorder = AsyncRecording(path, pending_limit=2)
        try:
            with NativePosePipe(calibration, directory / "native.log") as estimator:
                for frame in source.frames[:80]:
                    pair = [
                        (source.directory / name).read_bytes() for name in (frame.left, frame.right)
                    ]
                    started = time.monotonic()
                    result = estimator.estimate(
                        pair, frame.simulation_timestamp_ns, stopping, timeout_seconds=10
                    )
                    latencies.append(time.monotonic() - started)
                    received.append(
                        {
                            "timestamp_ns": frame.simulation_timestamp_ns,
                            "estimated_seconds": result["timestamp_seconds"],
                            "position_valid": result["position_valid"],
                        }
                    )
                    recorder.offer(result)
            status = recorder.close(timeout_seconds=0.05)
        finally:
            release.set()
            recorder.close()
    matching = all(
        abs(row["estimated_seconds"] - row["timestamp_ns"] / 1e9) < 1e-6 for row in received
    )
    passed = (
        len(received) == 80
        and matching
        and entered.is_set()
        and status["dropped"] > 0
        and any(row["position_valid"] for row in received)
        and (
            ("Injected pose recording write failure" in (status["error"] or ""))
            if fault == "enospc"
            else not status["writer_finished"]
        )
    )
    results[fault] = {
        "passed": passed,
        "received": received,
        "recording": status,
        "maximum_native_wall_seconds": max(latencies),
    }
passed = all(result["passed"] for result in results.values())
write_json_atomic(root / "async-faults/report.json", {"passed": passed, "faults": results})
print(json.dumps({"passed": passed, "report": "/workspace/outputs/async-faults/report.json"}))
raise SystemExit(0 if passed else 1)
