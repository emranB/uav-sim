# Release checks

The desktop and simulation services are independently buildable. A readiness response
is not a flight result, and an existing image is not evidence of a clean rebuild.

## Source and component checks

- All 386 Python source and script files pass the configured formatter and linter.
- Isolated Linux containers: 346 unit checks and 101 component checks passed.
- Five ROS-dependent component checks were skipped in the isolated image; they are
  not counted as passes. Real ROS connectivity was checked separately at runtime.
- Base, Linux and headless Compose configuration validation passed.

## Runtime evidence

On 18 November 2026, the clean release source `e59d76e` built all three image
targets: runtime, QGroundControl and ROS 2. A separate CPU/headless Compose project
then reached real-simulator readiness with two connected vehicles, recording available,
ROS connectivity and an advancing simulation clock. Both vehicles remained disarmed.
Embedded image source hashes matched the clean checkout.

This checks fresh-image build and headless startup. It is not a new GUI or flight
acceptance run. Windows Docker Desktop was not running during this cleanup; its Compose
configuration was checked, but Windows display startup was not rerun.

The featured waterfront recording retains the complete timeline, with labelled
speed changes. Its two aircraft returned and disarmed. It uses visual localization
and measured range obstacles; geographic meshes and source licences must be prepared
separately. Raw recordings and detailed logs remain outside the source repository.

## Reproduce checks

```sh
ruff check src scripts
ruff format --check src scripts
PYTHONPATH=src python -m unittest discover -s src/testing/unit -v
python scripts/dev/test_component.py
docker compose -f compose.yaml config --quiet
docker compose -f compose.yaml --env-file configs/runtime/headless.env up --build
```

Unit tests that use POSIX process behavior require Linux or a Linux container.
Component checks require the runtime image. Display and GPU configurations require
target-host checks; an unavailable dependency must be reported as skipped or failed.
The control API remains loopback-only and has no public-network authentication.
