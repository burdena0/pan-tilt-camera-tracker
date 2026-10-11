# Neural Pan/Tilt Camera Tracker

A two-axis USB-camera tracker with trained day/night vision, ESP32 motion control and a MuJoCo simulation built from the original printable parts. The camera mounts on the tilt shaft between the uprights.

## What runs where

- The Windows PC reads the USB camera and runs the neural detector, bearing-coordinate kalman tracker, global multi-object assignment and image-based camera servo.
- The ESP32 generates step pulses, enforces travel and speed limits, handles homing/manual override and checks every vision command with a CRC, session ID, sequence number and expiry time.
- MuJoCo simulates the original printable parts, both driven joints, the mounted camera and moving drone/bird geometry. Training labels and evaluation truth are kept out of the tracking controller.
- A spotlight follows the same pan/tilt assembly. Night processing adds contrast normalization, trained crop verification and forward/backward optical-flow proposals. Short-shutter rendering and camera-setting read-back make blur and exposure testable.

This uses established vision and robotics methods. It is a portfolio implementation, not a certified industrial product. The included learned model is trained on simulator images; real drone recognition and physical hardware commissioning are not yet verified.

## Files to start with

- `vision/src/camera_tracker/`: detector, training, camera calibration, tracking, camera control, USB capture, serial link and simulation.
- `simulation/assets/original/`: all five original STL files, preserved with hashes and attribution.
- `main/`: camera-only ESP-IDF firmware.
- `reports/`: recorded training, simulator, fault-injection, videos and model-export checks.
- `docs/SETUP.md`: installation and launch commands.
- `docs/VISION_REQUIREMENTS.md`: scope and evidence needed for completion.
- `docs/HOW_IT_WORKS.md`: the plain-language explanation.
- `docs/LIGHTING_AND_BLUR.md`: what helps blur/lighting and what still needs hardware measurements.
- `docs/RESULTS.md` and `docs/MODEL_CARD.md`: all measured cases, model data and known failures.

## Hardware

Classic ESP32-WROOM DevKitC, two NEMA 17 motors, DRV8825 drivers at 1/16 microstepping, four NC end stops, an NC E-stop loop, joystick, TRACK switch, HOME and STOP buttons. The USB camera connects to the Windows PC, not to the ESP32.

Motor pulses run from a 20 kHz GPTimer ISR. The planner limits acceleration to 80 deg/s² pan and 55 deg/s² tilt, with maximum speeds of 45 and 30 deg/s. After camera/serial loss the vision session latches off, commands return to zero and the planner decelerates to a holding stop. E-stop or conflicting limit switches disable motor outputs and latch a fault.

## Original mechanics

[Turret Mechanism by PJohnso1](https://www.thingiverse.com/thing:4259522) is licensed under [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/). Original print geometry is included with attribution in `simulation/assets/ATTRIBUTION.md`. The author's software reference is [PAJohnson/Turret](https://github.com/PAJohnson/Turret).

Simulation assembly uses the STL geometry and estimated joint/mount transforms. Printed-part geometry is exact to the downloaded files; pinion placement, camera adapter, masses, friction and motor dynamics are assumptions. Simulation success does not prove an assembled camera tracker works.
