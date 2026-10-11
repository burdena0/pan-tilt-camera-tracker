# Portfolio status

The camera-only source, trained models and closed-loop simulator are implemented. The payload module and firing interface have been removed. Both-axis simulation uses all five original printable STL files with a camera replacing the old load position and a spotlight on the same tilt assembly.

Configuration checks, the actual portable C protocol/watchdog and 36 Python tests passed locally. All four ONNX exports match their Torch checkpoints within 1e-4 logits on the sampled parity checks. 16 simulation cases and six recorded videos are included. `RESULTS.md` includes successful cases and the centering/defocus limits; one comparative night-performance check remains failed rather than being hidden. Exact-revision firmware and vision CI are attached to the published branch/PR.

Real camera accuracy and physical commissioning remain unverified. The neural model's training domain is synthetic simulator images; it must not be described as a validated real-world drone detector or a certified industrial system.
