# Model card

This is a small, locally trained synthetic perception baseline for a camera-only pan/tilt observer. It is not a pretrained commercial drone detector, certified industrial system or validated real-world model.

## Weights and input

The day and night pixel CNNs each contain 23,427 parameters and accept 320 x 192 RGB images. Each crop CNN contains 54,155 parameters and accepts 64 x 64 RGB crops. Classes are background, drone and bird. All four ONNX exports have SHA-256 manifests and their Torch checkpoints are included. CPU ONNX inference is checked against the trained weights.

Day proposals require .9 crop confidence. Night neural/contrast proposals require .75. Forward/backward optical-flow proposals require .9 and the previously verified class. Five consecutive observations confirm an angular track before initial lock. Model confidence is not a calibrated probability.

## Data and training

The initial daytime model ran 20 epochs on 1,600 training images, with 200 validation and 400 test scenes. Its bootstrap checkpoint was trained before geometric ray correction of renderer label artifacts. The published selected-daylight score uses corrected ray-checked test masks. The old dataset is cached locally and its provenance hash is recorded. Exact bootstrap reproduction requires the supplied checkpoint or the original cached dataset; fresh training uses the corrected annotation code.

A 12-epoch corrected-label adaptation and a further five-epoch lighting adaptation were trained. The lighting curriculum mixes 1,000 day and 600 night training images, 300 validation images and 600 test images. Initial scene seeds are 20000/40000/90000; added night seeds are 120000/140000/190000. Positions, orientations, object sizes, tones, gain, noise and blur vary. Data caches are ignored by Git because they are large and reproducible except for the explicitly identified bootstrap.

The day crop model trained for 24 epochs on 3,493 crops, with 427 validation and 844 test crops. The night crop model trained for 40 epochs on 4,984 crops, with 902 validation and 1,812 test crops. Night data includes actual detector proposals, jittered geometric boxes and hard background proposals. Matching boxes with IoU at least .4 receive the object class; IoU at most .1 is background; ambiguous boxes are excluded. Source scene splits stay separate. Crop contrast preprocessing is shared by training and runtime.

AdamW trains the networks. Validation chooses the best checkpoint. Model and tracker choices were also adjusted against the known development simulation cases. Therefore the recorded closed-loop cases are regression benchmarks, not an untouched blind generalization test.

## Held-out scores

| selected model | evaluation | result |
| --- | --- | --- |
| daylight pixel model | 400 corrected-mask synthetic images | drone IoU .7162, bird IoU .4465, foreground macro IoU .5814 |
| daylight crop model | 844 synthetic crops | accuracy 94.19%; accepted drone precision 98.57%, recall 84.10% at .9 |
| night pixel expert | 600 mixed synthetic images | scores are recorded in `reports/night_training.json` |
| night crop model | 1,812 proposal/jitter crops | accuracy 89.96%; accepted drone precision 95.44%, recall 82.26% at .75 |

These are different metrics on different tasks. Crop precision is not full-scene detector precision. A pixel IoU score is not a tracking accuracy percentage. Real drone precision/recall, false alarms per hour and range limits are unknown.

## Selection and failure evidence

The corrected-label adaptation had better pixel IoU but fragmented drone silhouettes and degraded tracking proposals. The original daytime checkpoint worked better in the closed-loop regression, so it remains selected. `reports/daylight_training.json` retains the adaptation report; `reports/selected_daylight_model.json` identifies the actual selected score and checkpoint. `reports/vision_training.json` describes the adaptation, not the selected deployment score.

An early combined night adaptation regressed daytime tracking. `reports/illumination_regression.json` retains that failed run. Separate day/night experts avoid that regression. Later crop contrast normalization, proposal training and verified optical flow improved night continuity. They do not solve the dim/defocus stress case or high-speed centering limit.

The only negative objects modeled are simple birds and background. Real clouds, branches, aircraft, insects, glare, rain, compression, motion patterns and drone shapes are not sufficiently represented. Collect real labeled recordings, split by session/location, calibrate the actual camera and measure latency before making real-world performance claims.
