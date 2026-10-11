# Commissioning checklist

- [ ] old payload removed, old servo/trigger wiring disconnected
- [ ] camera and adapter weighed, balanced and mechanically secured
- [ ] actual GPIO assignments/current limits checked against this build
- [ ] base fixed, both axes move freely, USB cable has slack at every limit
- [ ] NC E-stop/limit loops tested, including unplugged-wire behavior
- [ ] E-stop independently interrupts motor power
- [ ] home switch direction/backoff verified with no load first
- [ ] slow joystick and absolute moves confirm positive pan/tilt conventions
- [ ] camera focus/resolution fixed and measured ChArUco calibration passes
- [ ] preview model checked on separate real recordings, including birds and empty scenes
- [ ] TRACK switch must be seen off before motion sessions can enable
- [ ] STOP and joystick override cancel automated tracking
- [ ] unplugged USB/camera and stalled host stop commands and latch the session off
- [ ] re-enable is explicit; no restart after a fault or lost connection
- [ ] real centering error, latency, step loss, cable motion and thermal behavior measured

Simulation and compilation do not tick these physical checks automatically.
