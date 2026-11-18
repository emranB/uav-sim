"""Check explicit unavailable responses against the running local API; no flight commands."""

import argparse
import json
import urllib.error
import urllib.request
from pathlib import Path

from common.logging.storage import write_json_atomic


def request(url: str, body: dict | None = None) -> tuple[int, dict]:
    headers = {"Content-Type": "application/json", "X-Request-Source": "capability-validation"}
    command = urllib.request.Request(
        url,
        data=json.dumps(body).encode() if body is not None else None,
        headers=headers,
    )
    try:
        with urllib.request.urlopen(command, timeout=5) as response:
            return response.status, json.load(response)
    except urllib.error.HTTPError as error:
        return error.code, json.load(error)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://127.0.0.1:18080")
    parser.add_argument(
        "--report", type=Path, default=Path("outputs/reports/perception-capabilities.json")
    )
    arguments = parser.parse_args()
    checks = []
    try:
        status, fleet = request(f"{arguments.url}/vehicles")
        if status != 200 or not fleet.get("items"):
            raise RuntimeError("Running fleet inventory is unavailable")
        identifiers = [item["vehicle_id"] for item in fleet["items"]]
        for vehicle_id in identifiers:
            status, manifest = request(f"{arguments.url}/perception?vehicle_id={vehicle_id}")
            if status != 200 or manifest.get("vehicle_id") != vehicle_id:
                raise RuntimeError(f"Capability inventory failed for {vehicle_id}")
            for item in manifest["items"]:
                if item["implemented"]:
                    continue
                status, result = request(
                    f"{arguments.url}/perception/check",
                    {
                        "vehicle_id": vehicle_id,
                        "capability_id": item["key"],
                    },
                )
                checks.append(
                    {
                        "vehicle_id": vehicle_id,
                        "capability_id": item["key"],
                        "passed": status == 409
                        and result.get("code") == "CAPABILITY_UNAVAILABLE"
                        and result.get("vehicle_id") == vehicle_id
                        and result.get("accepted") is False,
                        "response": result,
                    }
                )
        status, group = request(
            f"{arguments.url}/perception/check",
            {
                "target_ids": identifiers,
                "capability_id": "object_detection",
            },
        )
        # Group operations preserve their existing HTTP-200 envelope with explicit failures.
        group_results = (
            group.get("results", {}) if len(identifiers) > 1 else {identifiers[0]: group}
        )
        checks.append(
            {
                "check": "collection_failure_details",
                "passed": status in {200, 409}
                and group.get("accepted") is False
                and len(group_results) == len(identifiers)
                and all(
                    item.get("code") == "CAPABILITY_UNAVAILABLE" for item in group_results.values()
                ),
            }
        )
        report = {"passed": bool(checks) and all(item["passed"] for item in checks)}
    except (OSError, ValueError, KeyError, TypeError, RuntimeError) as error:
        report = {"passed": False, "error": str(error)}
    report.update(
        {
            "schema_version": 1,
            "checks": checks,
            "scope": "unavailable reporting only; this is not perception or recovery validation",
        }
    )
    arguments.report.parent.mkdir(parents=True, exist_ok=True)
    write_json_atomic(arguments.report, report)
    print(json.dumps(report))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
