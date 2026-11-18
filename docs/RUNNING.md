# Setup and Docker

Run commands from the repository root. Use Docker Compose and Linux containers.
Initial builds download and compile PX4, Gazebo and ROS dependencies.

## Windows

Docker Desktop must be running with WSLg available for the visible workspace.

```powershell
Copy-Item .env.example .env
docker compose up --build
```

## Linux

Provide an X11/XWayland session and authorize the container user to access it.
Create writable `outputs/` and `outputs/qgroundcontrol/` directories for UID 1001.
Do not disable display access control globally.

```sh
cp .env.example .env
docker compose -f compose.yaml -f deploy/compose/linux.yaml up --build
```

Add `deploy/compose/xauthority.yaml` for cookie-based authorization, with
`XAUTHORITY_PATH` pointing to a private readable cookie file.

## Headless

```sh
docker compose -f compose.yaml --env-file configs/runtime/headless.env up --build
```

## Optional deployments

Keep root `compose.yaml` first when combining overlays from `deploy/compose/`.
GPU rendering needs host drivers and the NVIDIA Container Toolkit; CPU is the default.
Inspection, mapping, visual navigation, native recording and replay have separate
overlays. Select a consistent set with `COMPOSE_FILE` in your ignored `.env`.

The waterfront scene also needs prepared terrain, building meshes, aircraft assets
and their source notices. A Git checkout does not include downloaded geographic data.
The geography tools are `integrations.vision.terrain`, `integrations.geography.buildings`
and `integrations.geography.surface`; use their `--help` inside the optional geography
image. Provincial elevation and municipal building data must share the declared ENU
origin. Preserve their licences, metadata and hashes with derived assets.

Blender is optional and runs offline. Replay overlays require its retained executable
and revision-matched aircraft assets. The native desktop recording image adds FFmpeg;
the waterfront desktop image also adds Qt Multimedia. These optional profiles need
their own image builds; the base image alone does not provide the full demo.

## Check and stop

```sh
docker compose ps
docker compose logs -f control-panel gazebo
docker compose down
```

The control API is bound to `127.0.0.1:18080` and has no public-network authentication.
Persistent outputs survive container removal. Avoid `down -v` unless you intend to
remove runtime volumes. A separate mock stack is for component work, not flight proof:

```sh
docker compose --project-directory . -p uav-sim-mock -f deploy/compose/mock.yaml up --build
```
