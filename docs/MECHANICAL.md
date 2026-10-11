# Mechanical setup

The actual Thingiverse print meshes are used in the simulator. The stationary base stays fixed, the top cradle rotates about Z, and the shaft/camera assembly tilts about negative Y. The virtual camera lens is at approximately [48, 0, 157.2] mm at zero pan/tilt, on the shaft load mount between the uprights.

These mounting transforms are inferred from the STL bearing openings, shaft length and visible assembly. There is no dimensioned assembly file. Confirm the real bearing center, camera adapter clearance and optical alignment before using them as measured values. Extra motors, the adapter and camera casing are simplified geometry.

The old payload is removed from the design. The static validator assumes a 150 g camera/light load with a 25 mm center-of-mass offset: a 100 g camera plus an assumed 50 g lamp/adapter allowance. Weigh the actual assembly and measure the offset. The assumed mass does not include every printed component. Holding-torque calculations do not measure dynamic torque, backlash, missed steps or structural stiffness.

The simulated spotlight sits above the camera and points along the same optical direction. Both attach to the tilt assembly, so the lamp follows pan and tilt. Its cone and attenuation are renderer parameters, not a measured LED power/range specification.

Secure the base and route the USB cable so pan/tilt travel cannot pull it tight. Check the full travel with motors disconnected first. Align the lens with the mount's forward direction. The bearing tracker assumes pan/tilt home coordinates match that optical convention; it cannot estimate a bent or misaligned bracket.
