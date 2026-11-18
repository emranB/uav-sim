"""Machine-readable mock readiness check. This is not an end-to-end flight test."""

import json
import os
import sys
import urllib.request

try:
    with urllib.request.urlopen(
        "http://localhost:" + os.environ.get("BACKEND_PORT", "18080") + "/health", timeout=5
    ) as response:
        result = json.load(response)
    assert result["status"] == "ready" and result["mode"] == "mock"
except Exception as exc:
    print(json.dumps({"check": "mock-readiness", "passed": False, "error": str(exc)}))
    sys.exit(1)
print(json.dumps({"check": "mock-readiness", "passed": True, "flight_validated": False}))
