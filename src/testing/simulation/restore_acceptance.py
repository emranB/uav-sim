"""Native isolated idle/active/paused restore checks; never target the standard API."""

import argparse
import hashlib
import json
import time
import urllib.error
import urllib.request
from pathlib import Path

from common.logging.storage import write_json_atomic


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--recordings", type=Path, required=True)
    parser.add_argument("--port", type=int, default=18086, choices=[18086])
    arguments = parser.parse_args()
    arguments.output.mkdir(parents=True, exist_ok=True)
    ledger_path = arguments.output / "restore.json"
    if ledger_path.exists():
        parser.error("Use a new ledger; failed restore cases cannot be resumed")
    ledger = {
        "schema_version": 1,
        "passed": False,
        "clean_first_attempt": False,
        "cases": {name: {"state": "unrun"} for name in ("idle", "active", "paused")},
    }
    journal = arguments.output / "commands.jsonl"

    def save() -> None:
        write_json_atomic(ledger_path, ledger)

    def request(route: str, body: dict | None = None, expected_status: int = 200) -> dict:
        command = urllib.request.Request(
            f"http://127.0.0.1:{arguments.port}/{route}",
            json.dumps(body).encode() if body is not None else None,
            {"Content-Type": "application/json"},
        )
        try:
            with urllib.request.urlopen(command, timeout=35 if body is not None else 4) as response:
                status, result = response.status, json.load(response)
        except urllib.error.HTTPError as error:
            status, result = error.code, json.load(error)
        if body is not None:
            with journal.open("a") as stream:
                stream.write(
                    json.dumps(
                        {
                            "route": route,
                            "body": body,
                            "status": status,
                            "result": result,
                            "wall_time": time.time(),
                        }
                    )
                    + "\n"
                )
        if status != expected_status:
            raise RuntimeError(
                f"{route}: expected HTTP {expected_status}, received {status}: {result}"
            )
        return result

    def vehicles() -> list[dict]:
        return request("vehicles")["items"]

    def safe_ready() -> list[dict]:
        rows = vehicles()
        if [row["vehicle_id"] for row in rows] != ["vehicle_1", "vehicle_2"]:
            raise RuntimeError("Restore acceptance requires the explicit Atlas/Boreal fleet")
        if not all(
            row["ready"] and row["armed"] is False and row["recording"]["available"] for row in rows
        ):
            raise RuntimeError("Both native vehicles must be ready, disarmed and recording")
        return rows

    def hashes() -> dict:
        paths = []
        for folder in ("missions", "experiments", "configs"):
            paths.extend(
                path for path in arguments.recordings.rglob(f"{folder}/*") if path.is_file()
            )
        paths.extend(arguments.recordings.rglob("*/named/*.json"))
        fleet = arguments.recordings / "fleet.json"
        if fleet.is_file():
            paths.append(fleet)
        for manifest_path in (arguments.recordings / "replay").glob("capture-*/manifest.json"):
            manifest = json.loads(manifest_path.read_text())
            paths.extend(manifest_path.parent / name for name in manifest["files"])
            paths.extend((manifest_path, manifest_path.parent / "sealed.json"))
        return {
            str(path.relative_to(arguments.recordings)): hashlib.sha256(
                path.read_bytes()
            ).hexdigest()
            for path in paths
            if path.is_file()
        }

    def hover_plan(row: dict) -> dict:
        latitude, longitude = row["latitude_deg"], row["longitude_deg"]
        return {
            "fileType": "Plan",
            "version": 1,
            "groundStation": "QGroundControl",
            "mission": {
                "version": 2,
                "firmwareType": 12,
                "vehicleType": 2,
                "plannedHomePosition": [latitude, longitude, 20],
                "items": [
                    {
                        "type": "SimpleItem",
                        "command": command,
                        "frame": 3,
                        "autoContinue": True,
                        "doJumpId": index,
                        "params": [hold, 0, 0, None, latitude, longitude, altitude],
                    }
                    for index, (command, hold, altitude) in enumerate(
                        ((22, 0, 3), (16, 90, 3), (21, 0, 0)), 1
                    )
                ],
            },
        }

    def final_capture(identifier: str) -> dict:
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            payload = request("replay")
            if payload["capture"]["state"] not in {"capturing", "finalizing"}:
                row = next(item for item in payload["items"] if item["capture_id"] == identifier)
                return {"capture": payload["capture"], "inventory": row}
            time.sleep(0.1)
        raise TimeoutError("Capture did not finalize")

    save()
    try:
        for name in ledger["cases"]:
            case = ledger["cases"][name]
            case.update(state="running", started_wall_time=time.time())
            save()
            before = safe_ready()
            case["previous_generations"] = {row["vehicle_id"]: row["run_id"] for row in before}
            if name == "idle":
                plan = hover_plan(before[0])
                request("mission", {**plan, "vehicle_id": "vehicle_1"})
                request("missions/save", {"name": "restore-proof", "vehicle_id": "vehicle_1"})
                request("experiments/save", {"name": "restore-proof", "vehicle_id": "vehicle_1"})
                capture = request("replay/start", {"target_ids": ["vehicle_1", "vehicle_2"]})
                time.sleep(1)
                case["capture_id"] = capture["capture_id"]
            else:
                plan = hover_plan(before[0])
                request("mission", {**plan, "vehicle_id": "vehicle_1"})
                request("start", {"vehicle_id": "vehicle_1"})
                deadline = time.monotonic() + 60
                while time.monotonic() < deadline:
                    current = vehicles()[0]
                    if current["armed"] and current["relative_altitude_m"] >= 1.5:
                        break
                    time.sleep(0.2)
                else:
                    raise TimeoutError("Native hover mission did not become airborne")
                capture = request("replay/start", {"target_ids": ["vehicle_1"]})
                time.sleep(1)
                before_stop = vehicles()[0]
                request("replay/stop", {})
                stopped = final_capture(capture["capture_id"])
                after_stop = vehicles()[0]
                if (
                    after_stop["armed"] is not True
                    or not after_stop["recording"]["available"]
                    or after_stop["recording"]["path"] != before_stop["recording"]["path"]
                ):
                    raise RuntimeError(
                        "Midflight replay Stop changed flight/diagnostic writer state"
                    )
                case["midflight_capture_stop"] = {
                    "armed_after": after_stop["armed"],
                    "recording_available": after_stop["recording"]["available"],
                    "diagnostic_path_unchanged": True,
                    **stopped,
                }
                if name == "paused":
                    request("pause", {"vehicle_id": "vehicle_1"})
                    time.sleep(1)
                    first = request("telemetry?vehicle_id=vehicle_1")
                    time.sleep(0.5)
                    second = request("telemetry?vehicle_id=vehicle_1")
                    if (
                        not second["simulation_paused"]
                        or first["messages"]["LOCAL_POSITION_NED"]["time_boot_ms"]
                        != second["messages"]["LOCAL_POSITION_NED"]["time_boot_ms"]
                    ):
                        raise RuntimeError("Paused airborne clock was not observed")
                    case["paused_airborne_observed"] = True
            protected = hashes()
            case["persistent_hashes_before"] = protected
            admitted = request("restore/start", {})
            case["restore_id"] = admitted["restore_id"]
            request("restore/start", {}, 409)
            request("pause", {"vehicle_id": "vehicle_2"}, 409)
            deadline = time.monotonic() + 180
            while time.monotonic() < deadline:
                progress = request("restore")["restore"]
                case["restore"] = progress
                save()
                if not progress["active"]:
                    break
                time.sleep(0.25)
            else:
                raise TimeoutError("Native Master Reset did not finish within 180 seconds")
            if progress["state"] != "completed":
                raise RuntimeError(progress.get("error", "Native restore failed"))
            after = safe_ready()
            if any(
                row["run_id"] == case["previous_generations"][row["vehicle_id"]] for row in after
            ):
                raise RuntimeError("Native restore reused a controller generation")
            after_hashes = hashes()
            if any(after_hashes.get(path) != digest for path, digest in protected.items()):
                raise RuntimeError(
                    "Saved mission/configuration/history bytes changed during restore"
                )
            case["persistent_hashes_after"] = after_hashes
            if progress["defaults"]["target_ids"] != []:
                raise RuntimeError("Restore did not declare empty quick-action targets")
            if name == "idle":
                stopped = final_capture(case["capture_id"])
                if (
                    stopped["capture"]["state"] != "interrupted"
                    or not stopped["inventory"]["retained"]
                ):
                    raise RuntimeError("Master Reset did not retain interrupted capture evidence")
                case["interrupted_capture"] = stopped
            case.update(state="passed", finished_wall_time=time.time())
            save()
        ledger.update(passed=True, clean_first_attempt=True)
    except (OSError, RuntimeError, ValueError, KeyError) as error:
        ledger["error"] = str(error)
        for case in ledger["cases"].values():
            if case["state"] == "running":
                case.update(state="failed", error=str(error))
        save()
    finally:
        # Existing safe Stop/Land remains the only flight recovery authority.
        try:
            for row in vehicles():
                if row["armed"]:
                    request("stop", {"vehicle_id": row["vehicle_id"]})
            deadline = time.monotonic() + 60
            while time.monotonic() < deadline:
                rows = vehicles()
                if all(row["connected"] and row["armed"] is False for row in rows):
                    break
                time.sleep(0.25)
            ledger["cleanup_disarmed"] = all(
                row["connected"] and row["armed"] is False for row in rows
            )
        except (OSError, RuntimeError, KeyError) as error:
            ledger["cleanup_error"] = str(error)
        save()
    return 0 if ledger["passed"] and ledger.get("cleanup_disarmed") else 1


if __name__ == "__main__":
    raise SystemExit(main())
