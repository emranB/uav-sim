# Missions, faults and recordings

## Fly a mission

1. Wait for the selected vehicles to be ready, with fresh telemetry and required sensors.
2. Load or plan a mission. QGroundControl displays the plan; the acknowledged backend
   navigation session owns flight when using the visual mission controls.
3. Select Atlas, Boreal or both in the command-target selector. Inspect changes the
   viewed vehicle; it does not change an already dispatched command.
4. Start the mission and watch each vehicle's acknowledgement and health.
5. Request landing when needed. Wait for confirmed landing and disarm before reset.

Only supported controls are enabled. World pause affects both vehicles and can make
live telemetry stale. Reset clears runtime faults while retaining saved records.
Do not issue competing flight commands from QGroundControl and the backend.

## GPS loss and perception

Select fault targets explicitly and wait for acknowledgement before another request.
Requested GPS input, actual fix and measured GNSS fusion are separate states. A click
or a red input badge does not by itself prove that GNSS fusion stopped.

Visual localization requires healthy camera-derived poses and a consistent reference.
The stereo-map-reference option keeps the mission reference stable across expected
GNSS resets. Stale or unhealthy observations can stop navigation and trigger landing.

The waterfront demo uses camera localization and measured horizontal LiDAR ranges
for obstacles. Other profiles support stereo or fused obstacle observations. Unknown
space stays unknown. Ground truth is used for scoring and replay, not navigation.

The reconstruction view shows tracked landmarks, surfaces and estimated paths.
Frozen or stale data is labelled; a displayed map is not proof of localization accuracy.

## Capture and replay

Start Capture before the part you want to keep. The selected vehicle set is latched
for that capture. Stop seals the interval without stopping a mission or diagnostic
logging. Keep/Save protects it from routine retention cleanup.

With native recording enabled, the desktop records real application pixels. The
optional worker uses sealed timestamps and recorded poses to render Blender views.
The ordinary profile queues on Stop; profiles configured with `UAV_REPLAY_RENDER_TRIGGER=keep`
queue on Keep. Repeated requests reuse the capture's automatic attempt.

Rendering is offline. The UI reports queued, rendering, completed, cancelled or failed
jobs. Playback is available only after completion. Missing or mismatched screen evidence
is an error; the worker must not fabricate native footage. After the capture ends,
any retained UI tail labels replay poses as sealed rather than live.

A chase camera can inspect the recorded interval for presentation framing. It must
not reveal future motion or obstacle observations earlier in the replay.

## Storage

Keep durable missions, captures and reports under `outputs/`, outside container layers.
Idle telemetry and recording queues are bounded. Keep the current deliverables, one
recovery point, required source licences and useful failure evidence. Delete only
superseded derivatives that are no longer referenced. Do not put raw runs in Git.

For the demonstrated measurements and geographic limitations, see
[demo notes](media/DEMO-NOTES.txt). For current verification, see
[release checks](RELEASE_CHECKS.md).
