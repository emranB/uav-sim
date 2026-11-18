# Architecture

## Components

The desktop composes Qt views and asynchronous clients. Application services own
missions, readiness, recovery and recording; widgets express user intent. Panels do
not call one another. Shared contracts stay independent of Qt and concrete transports.

Gazebo owns physics, world geometry and simulated sensors. PX4 SITL estimates and
controls each vehicle. QGroundControl provides planning and native vehicle views.
MAVLink is the command transport; each operation has one declared control authority.
ROS 2 Jazzy and the XRCE-DDS bridge provide telemetry and simulation-clock observation.

Each vehicle has explicit application and MAVLink identities, endpoints and recording
generations. Do not silently select the first vehicle or share mutable navigation state.
Lifecycle transitions reject conflicting operations and report acknowledgement separately
from process liveness and vehicle readiness.

## Data and time

Adapters translate protocol messages into application models. Coordinate conversions
happen at tested boundaries: world and navigation positions use ENU metres; native
frames and quaternion conventions are recorded explicitly. UTC receipt time and
simulation time are distinct. Reset epochs invalidate stale observations.

ROS uses explicit domain/QoS configuration and firmware-matched message definitions.
SITL normally disables XRCE agent time correction and uses the simulation clock.
A paused clock must not prevent wall-clock discovery or safe recovery.

Camera adapters publish paired measurements through bounded in-memory paths. Estimation,
map construction and command execution stay separate. The observed-space planner uses
conservative inflation and uniform-cost grid search; unmapped cells are not free.
Freshness and source-health checks remain mandatory during motion.

## Recording and rendering

Capture freezes source metadata, selected vehicles, timing and relevant assets.
Background diagnostics and manual capture have separate lifecycles. Native screen
recording seals encoder timestamps and checksums before composition.

Blender and FFmpeg run in optional bounded workers outside the control loop. Replay
preserves actual pose timing and distinguishes estimated state from simulator truth.
Cancellation stops owned children; failures remain visible and preserve the source.

## Repository

- `src/desktop`: startup and presentation composition.
- `src/common`: application contracts, lifecycle, navigation and storage.
- `src/integrations`: PX4, MAVLink, ROS/DDS, perception and replay adapters.
- `src/gazebo`, `src/qgroundcontrol`: simulator/view integration.
- `src/control_panel`: feature panels and common presentation.
- `src/testing`: unit, component and real-stack checks.
- `configs`: declarative vehicles, worlds, missions and runtime inputs.
- `deploy/compose`: optional deployment overlays.
- `outputs`: ignored generated and durable run data.

The local HTTP API is loopback-only. It is not an authenticated public control service.
