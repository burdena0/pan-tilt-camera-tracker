# Pan/Tilt Camera Tracking Simulation

A MuJoCo simulation of a two-axis camera tracking moving drone geometry. The camera moves with the pan/tilt assembly, and its rendered images feed trained day/night vision models and a tracking controller.

The project currently runs in simulation only. The models use synthetic training data; real-camera detection and physical hardware operation have not been tested.

## Simulation

The scene uses the original printable STL geometry. The detector classifies drone, bird and background pixels, then checks candidate crops. A Kalman tracker maintains object IDs and compensates for camera rotation. The controller drives both simulated joints to center the selected target.

Scenarios cover distractors, occlusion, lost frames, corrupt commands, low light and motion blur. The simulation also exercises the C command parser and watchdog through a host build. A spotlight follows the camera for the night scenarios.

## Repository

- `vision/src/camera_tracker/`: rendering, detection, tracking, training and benchmarks.
- `vision/tests/`: camera geometry, rendering, tracking, model and command-protocol tests.
- `simulation/assets/`: original STL files and prepared simulation meshes.
- `reports/`: recorded synthetic results, frame traces, plots and videos.
- `main/`: ESP32 firmware source for a future physical build; hardware operation is unverified.
- `tools/`: mesh preparation, host builds and model checks.

## Geometry

The print geometry comes from [Turret Mechanism by PJohnso1](https://www.thingiverse.com/thing:4259522), licensed under [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/). See [asset attribution](simulation/assets/ATTRIBUTION.md) for the source files and simulation modifications.

Joint placement, the camera adapter, masses, friction and motor response are estimates.
