# Architecture

Camera -> latest-frame capture -> day/night neural pixel classification -> contrast/optical-flow proposals when dark -> whole-crop neural verification -> calibrated angular bearings -> kalman/global assignment -> persistent target ID -> bounded two-axis camera velocity -> CRC/sequence/session/age checks -> acceleration-limited step planner.

The moving camera's pan and tilt are used in the bearing transform. This keeps camera rotation out of the target velocity estimate. Only angular direction is estimated: a single camera does not supply measured target distance here.

Firmware tasks:

- 20 kHz step ISR: step pulses and immediate directional hard-stop checks.
- 100 Hz planner: braking/acceleration-limited position and velocity control.
- 5 ms safety task: latched E-stop and contradictory switch faults.
- 10 ms vision watchdog: expires stale commands and cancels the vision session.
- 20 ms telemetry: position, homing, session, accepted sequence and faults with CRC.
- 10 ms manual task: joystick override, HOME and STOP.
- Serial console: bounded framed packets and local setup commands.

Tracking motion requires homed axes, a healthy E-stop loop, no safety faults, TRACK selected after it has been seen off and a fresh explicit session. Joystick movement cancels that session. Stale or rejected packets cancel it too. A normal session stop zeros the command and decelerates while holding position; an electrical/safety fault disables the drivers.

The Python runtime never queues a backlog of camera frames. It processes the newest completed frame, rejects stale telemetry/frame age and keeps the serial write deadline bounded. The firmware independently validates the capture time in device milliseconds. CRC is error detection, not cryptographic authentication; this is a local USB protocol, not an internet control interface.
