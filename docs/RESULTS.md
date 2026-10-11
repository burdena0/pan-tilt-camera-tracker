# Recorded simulation results

These are synthetic scenes at 320 x 192, 20 camera frames per second, 12 seconds each. The controller receives RGB images and joint telemetry; it does not receive simulator object positions or labels.

| case | mean error (px) | p95 (px) | observed lock / frames | possible bird handover frames |
| --- | ---: | ---: | ---: | ---: |
| fixed_camera | 44.40 | 63.31 | 218/240 | 0 |
| nominal | 2.04 | 5.00 | 219/240 | 0 |
| distractors | 2.09 | 5.00 | 216/240 | 0 |
| occlusion | 2.83 | 6.87 | 210/240 | 0 |
| capture_stall | 2.95 | 5.67 | 206/240 | 0 |
| corrupt_packet | 3.04 | 5.72 | 217/240 | 0 |
| stale_frame | 3.04 | 5.72 | 217/240 | 0 |
| negative | n/a | n/a | 0/240 | 0 |
| blur_dim | 30.46 | 87.80 | 52/240 | 0 |
| distant | 2.08 | 6.96 | 194/240 | 0 |
| night_long_shutter | 3.80 | 12.14 | 220/240 | 9 |
| night_spotlight | 3.78 | 11.25 | 221/240 | 4 |
| night_short_shutter | 4.32 | 9.19 | 195/240 | 0 |
| night_spotlight_short_shutter | 3.98 | 11.07 | 200/240 | 0 |
| night_spotlight_short_shutter_negative | n/a | n/a | 0/240 | 0 |
| night_fast_spotlight_short_shutter | 19.19 | 45.24 | 217/240 | 0 |

Daytime mean error drops 95.41% against the fixed-camera baseline. Nominal pan spans 29.12 degrees and tilt spans 8.48 degrees.

Centering averages include every visible, enabled evaluation frame after the first second, even when detection is missing. Fault cases stop accumulating this centering metric after the session disables. Observed locks can still be reported in later frames while motion is latched off.

Possible bird handover is a geometric proximity diagnostic, not an independently measured identity metric. Overlapped drone/bird images can make it ambiguous. The day and short-shutter bird-only cases have zero camera locks.

The lamp brightens target pixels and a separate short-shutter test preserves moving edges. The combined lamp/2 ms case does not beat the lamp-off/40 ms case on mean centering error. That performance gate fails; the failure is retained. The shorter exposure receives more noise in the matched-gain sensor approximation.

Fast night motion reaches the 45 deg/s pan cap and has much larger error. The dim/defocus stress case also loses tracking. These are unresolved performance limits, not successful cases.

CPU inference on the recorded Windows machine: median 7.94 ms, p95 13.51 ms. This excludes capture, rendering, motor motion and USB transport.

The shutter averages five temporal samples. Lamp brightness is a Phong-renderer value, not calibrated lux or electrical wattage. The camera noise is normalized Gaussian noise, not a measured camera photon model. Motor inertia, friction, bracket transforms and drive response are estimates. No physical fit, real-drone accuracy or live hardware result is implied.

Raw values, frame traces, export hashes, regression runs and videos are in `reports/`. Run `tools/summarize_results.py` after the benchmark to regenerate this table.
