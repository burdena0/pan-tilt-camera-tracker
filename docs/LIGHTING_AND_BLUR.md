# Lighting and blur

The spotlight is mounted above the camera on the same tilt assembly. Its beam follows both joints. A cone limits coverage and attenuation reduces its effect farther away. MuJoCo's Phong lighting values are renderer units, not LED wattage or measured lux. This does not size a real lamp for an outdoor drone at an arbitrary distance.

## Why the light and shutter work together

Motion blur depends on image speed multiplied by exposure time. At 500 pixels per second, 20 ms gives about 10 pixels of smear; 2 ms gives about 1 pixel. Shorter exposure also collects less light, so illumination and sensor sensitivity matter.

The night benchmark compares 40 ms without a lamp, 40 ms with the spotlight, 2 ms without a lamp and 2 ms with the spotlight. It averages five rendered instants over the shutter interval and approximates camera motion from its instantaneous velocity. Shorter exposure gets more normalized noise at matched output gain.

This is an exposure/noise experiment, not a calibrated photon simulation. No manufacturer-specific sensor, rolling readout, measured vibration, optical defocus or lamp measurement is modeled. `blur_dim` remains a separate Gaussian-defocus stress test; a faster shutter cannot reverse a defocused lens.

## Software that helps

- Training uses varied brightness, noise and blur, plus a day/night curriculum with spotlight states.
- A second trained crop model checks the whole object's shape before camera lock.
- The night crop path smooths noise and normalizes contrast; separate day/night weights avoid regressing the daytime detector.
- Forward/backward optical flow stabilizes proposals between neural frames. Each flow box needs same-class neural verification at .9 confidence and periodic reseeding.
- Camera-rotation compensation and target-velocity feedforward reduce lag and relative image motion.
- Live logs include brightness, clipping and an edge-sharpness statistic. These are diagnostics, not calibrated focus measurements.
- A one-pixel blur budget is estimated in milliseconds from relative angular velocity and calibrated focal length.
- Requested exposure/gain settings require driver acceptance and matching read-back.

## The actual USB camera

Manual exposure, auto-exposure and gain values depend on the camera, Windows driver and DirectShow backend. Do not assume raw exposure values are seconds or every device has the same manual-mode flag. Use documented values, then verify shutter timing with moving-scene or hardware measurements.

The runtime accepts `--raw-auto-exposure`, `--raw-exposure` and `--raw-gain`. An exposure request requires an explicit manual-mode driver value. Read-back verifies the reported property, not physical shutter duration. The displayed blur budget is advice; it is not automatically converted into undocumented driver units.

Focus must match observation distance. A rigid bracket and balanced shaft reduce vibration. A global-shutter USB camera removes rolling-readout distortion, but still needs short exposure for motion blur. Continuous regulated illumination avoids an unsynchronized flashing lamp. None of these properties has been measured on the user's camera.

The lamp is implemented in simulation. The ESP32 has no LED PWM/trigger output or lamp command. A physical lamp needs a suitable driver, heatsink, supply and measured mount.

## Measured tradeoffs

`RESULTS.md` includes the four matched-motion night combinations and a faster night stress case. The spotlight test confirms brighter target pixels, and a moving-object shutter test confirms stronger edges with 2 ms versus 40 ms. Those are different measurements from centering error.

The combined light/short-shutter setup tracks in the normal night scenario. It does not outperform the 40 ms lamp-off baseline on mean centering error: the added short-exposure noise offsets some benefit. The stricter performance comparison is kept as a failed check in `reports/acceptance.json`. Defocus/dim lighting and faster motion also remain limitations. No claim is made that every blur or lighting problem is solved.

References: [MuJoCo light model](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-light), [OpenCV camera properties](https://docs.opencv.org/4.x/d4/d15/group__videoio__flags__base.html), [Basler shutter types](https://docs.baslerweb.com/electronic-shutter-types).
