# How the camera tracker works

The project is a camera on a moving stand. The stand turns left/right and tilts up/down to keep a detected drone near the middle of the image. The old payload and trigger interface are removed.

## The mechanical model

The simulation loads the original base, top cradle and tilt shaft STL files. It also uses split copies of the original gears and bushings. These are the actual downloaded printable surfaces, not a box drawn to resemble the mechanism.

The base stays still. The top cradle turns around a vertical axis, called **pan**. The shaft rotates around a horizontal bearing axis, called **tilt**. The camera is attached to that shaft's load mount, between the uprights. When the shaft tilts, the camera tilts with it. When the cradle pans, everything above the base pans with it.

The parts are in millimeters in their STL files. MuJoCo scales them to meters. Assembly transforms put the print-bed parts together. The tilt bearing height is inferred from the opening in the cradle. The camera bracket, motors, masses and friction are simplified estimates. This is a geometry-based simulation, not a measurement of the finished hardware.

## What the neural network sees

The model only receives the camera image. It never receives the simulator's drone position, target ID or correct answer.

It is a small encoder-decoder convolutional neural network. A convolution slides a learned pattern over the image. Early layers can learn edges and contrast. Later layers combine them into shapes. The encoder shrinks the image to collect context; the decoder expands it again to say what each pixel looks like.

Each pixel gets three scores: background, drone and bird. A softmax turns them into relative probabilities. Connected foreground regions become proposals. A second CNN checks each object's whole shape in a square crop, correcting mixed bird/drone fragments or rejecting background. Very small, clipped and low-confidence objects are rejected. Confidence is a model score, not a calibrated probability of correctness.

There are separate day and night weights. Average image brightness selects the night expert below 50 and switches back above 80, so small brightness changes do not keep flipping models. These values are image levels, not measured lux. The pixel network has 23,427 parameters and each crop network has 54,155. Only one day/night pair runs for a frame.

The night path also proposes compact contrast regions. It smooths a crop and stretches its shared RGB range before classification. This helps a dark silhouette look more consistent without pretending to recover detail that the camera never captured.

Optical flow adds short-term continuity. It follows image corners from the previous frame, then checks them backward. Corners that fail the backward check or disagree about movement are discarded. The resulting box still needs at least .9 crop confidence and the same class before use. The box is reseeded from neural geometry every 20 frames. Flow is an extra proposal, not permission to follow an unverified object indefinitely.

Training images come from the simulator, with randomized positions, orientations, object scales, brightness, noise and blur. The simulator can produce exact pixel labels, so no person needs to draw masks for those images. Training adjusts the model weights to make its predictions resemble the labels. Validation images choose the checkpoint. Separate harder test images measure it after training.

Renderer label candidates are checked with geometric rays so an occluder or ground cannot be mislabeled as a drone. The initial daylight checkpoint predates this label correction and is supplied explicitly; its published score uses corrected held-out masks. A later corrected-label adaptation improved pixel IoU but made worse closed-loop proposals. Both its report and the failed illumination regression are retained. The chosen daylight weights are the checkpoint that performed better in the development tracking scenarios.

This is real training of an actual network, but the data is synthetic. A good simulator score does not mean it will recognize every real drone, cloud, branch or bird. Real recordings must be labeled and tested separately. The image/mask manifest loader supports that and rejects duplicate images across splits.

## Why camera calibration matters

A pixel is a location on the image, not an angle in the room. The lens decides how many pixels represent a given angle. Calibration measures focal length, principal point and lens distortion.

The ChArUco board is a checkerboard with identifiable square markers. The calibration tool matches known board corners across different views and fits the camera model. It checks image coverage, duplicate files and reprojection error. Reprojection error asks: if the estimated lens model projects the known board points back onto the image, how far are they from the measured corners?

Live motion requires a measured calibration file. The simulator uses its known virtual lens parameters. Keep the real camera at the same resolution and focus as calibration.

## Remembering objects across frames

A detector answers “what is in this image?” a tracker answers “which object is this over time?”

Each track has an ID, a position estimate, an angular velocity estimate and uncertainty. A kalman filter predicts where the object should appear next, then corrects that prediction with the new measurement. It gives noisy measurements less influence when the estimate is already consistent.

The tracker works in angular bearing coordinates. It combines the measured pixel ray with the camera's pan/tilt angle. That removes camera rotation from the object's motion estimate. Without this step, turning the camera left makes everything in the image move right, and a tracker can mistake that for the drone moving.

When several detections exist, the tracker calculates which matches are plausible. A mahalanobis gate accounts for uncertainty, rather than just comparing raw distances. Hungarian assignment chooses the best collection of matches for the whole frame. Drone and bird labels do not match each other.

A track needs five consecutive observations before confirmation. The camera initially locks to a confirmed drone near its center. That ID persists. If it disappears, the camera stops requesting movement while briefly remembering the track. It does not silently jump to another object when the old ID expires. The operator can reset the lock.

Night box centers are noisier, so the night filter assumes .9 degree measurement noise and 45 deg/s² acceleration noise, versus .18 and 12 by default in daylight. These are tuned simulation assumptions. Overlapping night outliers cannot immediately create competing IDs inside an existing predicted silhouette. Genuinely overlapping objects can therefore take longer to separate into distinct tracks.

## The spotlight and blur changes

The light sits above the camera on the tilt assembly. Both turn together, so the light stays near the camera's view. The simulation gives it a cone and distance attenuation. Its brightness values are renderer units, not the power or range of a real lamp.

Motion blur is roughly image speed times shutter duration. An object moving 500 pixels per second smears about 10 pixels during 20 ms, but about one pixel during 2 ms. A shorter shutter collects less light. The night experiment adds more noise to the short exposure at matched gain to reflect that tradeoff.

The model integrates five rendered instants over each exposure. It is enough to test temporal smear, but it is not a calibrated sensor simulation. Optical defocus, rolling-shutter timing, vibration and real LED illumination are not measured. The separate defocus/dim test remains weak; a spotlight cannot refocus a lens.

The Windows runtime can request raw exposure, gain and auto-exposure values and require matching driver read-back. Those values depend on the actual camera and driver. It also logs brightness, clipping, edge sharpness and an estimated one-pixel shutter budget. None of that confirms actual exposure timing until measured on the real camera.

## Turning pixels into both-axis movement

The controller compares the selected track's bearing with the current camera angle. A positive pan error means turn left. A positive tilt error means tilt up.

The controller combines estimated target angular velocity with proportional centering correction. At a gain of 3, a 5 degree error adds 15 degrees per second to the velocity estimate. This follows motion without waiting for a large error to build. A small deadband reduces tiny corrections. Pan and tilt have separate speed caps and travel limits.

The firmware does not jump instantly to that speed. Its planner ramps speed according to acceleration limits. The step interrupt turns the planned speed into pulses for the two drivers. In simulation, the same bounded velocity request goes through the actual C protocol/watchdog, then a rate-limited virtual motor model drives the two joints.

## Keeping old data out of control

The capture thread keeps one latest frame, like a mailbox whose old letter is replaced. It does not build a long queue of images while inference catches up.

The host checks frame age and telemetry age before requesting movement. Firmware also checks the packet. Every packet carries a session ID, sequence number, capture time, two speeds, expiry time and CRC. The session ID keeps an old connection's commands out. The sequence number rejects duplicates/replays. The capture time rejects stale data. The CRC catches accidental corruption.

The firmware watchdog is independent of the Windows program. If the program stops sending fresh commands, the session latches off and requested speed returns to zero. The planner decelerates to a holding stop. Joystick motion takes over locally. STOP cancels tracking. An E-stop or contradictory limit switches disable motor outputs and invalidate the home reference, so the mechanism must be cleared and homed again.

CRC is not encryption or authentication. This is a local USB link, with no internet command interface.

## What the simulation proves

The benchmark renders the mounted camera's image, runs the trained model, tracks detections, generates movement requests and moves the joints. The changed joint angles change the next camera image. This feedback loop is the useful test: the controller has to keep correcting what it actually sees.

The simulator's labels are used only for training and evaluation. Evaluation measures how far the visible drone center is from the image center. It also records pan/tilt motion, locked IDs, missing detections, protocol rejection and watchdog state. A fixed-camera baseline shows whether movement improved centering.

Separate scenarios test distractors, temporary occlusion, missing camera frames, stale frames, corrupt packets, bird-only scenes, blur/dim lighting and distant objects. The videos show the actual assembly on the left, the camera image on the right and neural drone probability underneath.

The full numbers are in `RESULTS.md`. Daytime tracking is much better than holding the camera still. The light/short-shutter night case also follows on both axes, but the short shutter does not beat the 40 ms baseline on mean centering error in this particular noise model. Faster flight has larger error and approaches the pan speed cap. Those failures are part of the result, not removed from the report.

## What still needs the real build

The simulator cannot prove motor wiring, driver current, switch placement, USB cable clearance, mount rigidity, backlash, missed steps or real detector accuracy. Its motor torques and inertias are assumptions. The ESP32 reports step-count position, not an independent encoder measurement.

Camera timestamps are host receipt times, not hardware exposure timestamps. USB buffering can add delay that this timestamp does not reveal. Measure that delay and real centering error before claiming deployment performance.

The implementation uses established methods, but “industry standard” is not a certification earned by adding a neural network or passing a simulator. The useful portfolio claim is the system you implemented, the tests you actually ran, the measured results and the limits you can explain.
