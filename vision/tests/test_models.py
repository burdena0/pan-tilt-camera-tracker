import numpy as np
from camera_tracker.detection import NeuralDetector
from camera_tracker.simulation import ROOT
from camera_tracker.crops import normalize_contrast

def test_shipped_model_hashes_shapes_and_day_night_switching():
    detector=NeuralDetector(ROOT/'vision/models/synthetic_drone.onnx')
    for brightness,mode in [(200,'daylight'),(10,'night'),(65,'night'),(100,'daylight')]:
        detections,probabilities=detector.detect(np.full((192,320,3),brightness,np.uint8))
        assert detector.active_mode==mode
        assert probabilities.shape==(3,192,320) and np.isfinite(probabilities).all()
        assert all(np.isfinite(d.bbox).all() and 0<=d.score<=1 for d in detections)

def test_constant_dark_crop_does_not_divide_by_zero():
    normalized=normalize_contrast(np.full((3,64,64),7,np.uint8))
    assert normalized.shape==(3,64,64) and normalized.dtype==np.uint8
    assert not np.any(normalized)
