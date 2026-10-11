# Requested system and completion evidence

The current scope is a camera-only drone tracker. The old water payload is removed. The user's USB camera and Windows PC are the intended live platform. Simulation must use the original 3d print geometry, put the camera at the old load mounting position and show automated movement on both axes.

| requirement | authoritative evidence |
| --- | --- |
| camera-only conversion | no payload module, trigger pin or firing command in the compiled firmware |
| original printable geometry | all five source STL files, source/license attribution, hashes and loaded mesh assets |
| camera mounted on tilt assembly | simulator camera body is the tilt body, documented mounting transform and rendered view |
| actual neural model | trained weights, ONNX artifact/hash, disjoint data splits and held-out scores |
| drone/bird discrimination | independent held-out image results, bird-only and distractor simulation cases |
| persistent multi-object tracking | angular kalman state, gated Hungarian matching, confirmed IDs and identity tests |
| calibrated moving-camera coordinates | camera-rotation compensation and optical projection tests, measured-calibration live gate |
| automated two-axis camera motion | image-driven closed-loop simulator, pan/tilt traces, centering error and video |
| firmware command watchdog | actual C parser/guard called by simulation, malformed/replay/stale/wraparound tests |
| degraded-input behavior | occlusion, camera stall, corrupt packet, stale frame, blur/dim and distant-object cases |
| Windows runtime | bounded latest-frame USB capture, ONNX CPU inference, CRC serial telemetry and logs |
| reproducibility | installation/train/simulation commands, test suite, CI and model/data metadata |
| explanation and resume | plain-language walkthrough, limitations, current project title/order and corrected degree |
| physical/live performance | measured calibration, real held-out recordings and commissioning results; not implied by software tests |

Passing synthetic tests does not establish real drone detection accuracy. Use diverse real recordings, separate locations/sessions for train and test, and report precision/recall, false alarms, centering error and measured latency before describing the tracker as field-ready. There is no industrial certification claim.
