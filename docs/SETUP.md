# Setup

## Windows vision environment

From the repository root:

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -e './vision[simulation,test,report]'
.\.venv\Scripts\python.exe tools/prepare_meshes.py
.\.venv\Scripts\python.exe tools/build_guard.py
.\.venv\Scripts\python.exe -m pytest vision/tests -q
.\.venv\Scripts\python.exe -m camera_tracker.benchmark --video
.\.venv\Scripts\python.exe tools/summarize_results.py
```

`build_guard.py` needs GCC. It uses GCC on PATH, or the installed MSYS2 UCRT64 compiler on Windows. This builds the actual C command parser/watchdog for the simulator and runs its host checks.

## Train again

Install CPU PyTorch from its official wheel index, then the training extras:

```powershell
.\.venv\Scripts\python.exe -m pip install torch --index-url https://download.pytorch.org/whl/cpu
.\.venv\Scripts\python.exe -m pip install -e './vision[train]'
.\.venv\Scripts\python.exe -m camera_tracker.train --epochs 20
```

That command trains a new corrected-label baseline; it overwrites the deployment pixel model. The supplied selected daylight bootstrap predates ray-label correction. Its metadata is in `reports/selected_daylight_model.json` and `reports/data_lineage.json`. Cloning the included models needs no retraining. To rebuild the adaptation/crop pipeline using the supplied bootstrap checkpoints:

```powershell
.\.venv\Scripts\python.exe -m camera_tracker.train --epochs 12 --resume vision/models/bootstrap_synthetic.pt --learning-rate .0005
Copy-Item reports/vision_training.json reports/daylight_training.json
.\.venv\Scripts\python.exe -m camera_tracker.train --lighting-curriculum --epochs 5 --resume vision/models/lighting_bootstrap.pt --learning-rate .0005
.\.venv\Scripts\python.exe -m camera_tracker.train_classifier --epochs 24
.\.venv\Scripts\python.exe tools/bundle_illumination_models.py --restore-daylight
.\.venv\Scripts\python.exe -m camera_tracker.train_classifier --epochs 24 --dataset vision/output/dataset_ray_checked.npz
.\.venv\Scripts\python.exe tools/select_daylight_checkpoint.py
.\.venv\Scripts\python.exe tools/bundle_illumination_models.py
.\.venv\Scripts\python.exe tools/prepare_proposal_crops.py
.\.venv\Scripts\python.exe -m camera_tracker.train_classifier --epochs 40 --name crop_night --dataset vision/output/proposal_crops.npz --normalize-contrast --prepared-crops
.\.venv\Scripts\python.exe tools/verify_models.py
.\.venv\Scripts\python.exe -m camera_tracker.benchmark --video
.\.venv\Scripts\python.exe tools/summarize_results.py
```

Training, validation and test scenes use separate fixed seeds. Validation chooses checkpoints; test scenes are scored afterward. The lighting curriculum needs the corrected-label dataset. Proposal generation uses the five-pixel closing kernel from the recorded training experiment; inference uses seven pixels. Caches are under `vision/output/`. Runtime versions and cache hashes are recorded. Different renderers, optical-flow numerics or libraries can change pixels, trained weights and closed-loop traces.

## Measure the real camera

Make a ChArUco board image:

```powershell
.\.venv\Scripts\python.exe -m camera_tracker.calibrate --board --output board.png
```

Print with no resizing and measure each square as 25 mm. Collect at least 12 usable, varied views at the exact resolution/focus used for tracking. Calibration checks image coverage, duplicate files and reprojection error.

```powershell
.\.venv\Scripts\python.exe -m camera_tracker.calibrate --images calibration-images --output camera.json
.\.venv\Scripts\python.exe -m camera_tracker.runtime --calibration camera.json --camera 0
```

Preview mode has no serial output. The included synthetic model is a simulator baseline; inspect real recordings and collect real labeled footage before claiming real drone performance.

## Firmware and live camera motion

Use ESP-IDF v6.0.3 and review `main/turret_config.h` against the actual wiring. Build with `idf.py set-target esp32` then `idf.py build`. Do not flash an unreviewed wiring configuration.

Disconnect the old trigger/servo wiring and remove the old payload. Mount and balance the camera, secure the base and provide cable slack. With TRACK off, restore E-stop, clear faults and home both axes. Verify directions and end stops with slow manual moves first. After homing, select TRACK, close the monitor so the port is free, then run:

```powershell
.\.venv\Scripts\python.exe -m camera_tracker.runtime --calibration camera.json --camera 0 --port COM5 --live
```

Replace COM5 with the actual port. Press `c` to clear a restored fault with TRACK off, `h` to home with TRACK off, `e` to enable with TRACK selected, `r` to reset target lock, space to stop and `q` to quit. Joystick movement overrides tracking. STOP cancels the session. Stale/corrupt packets or watchdog timeout require explicit re-enable. E-stop and limit conflicts invalidate home and require rehoming.

The spotlight is simulated; there is no live lamp output in the firmware.

Camera timestamps are host receipt times, not sensor exposure timestamps. USB-driver buffering and motor step loss are not measured by this implementation. Physical validation needs those measurements.
