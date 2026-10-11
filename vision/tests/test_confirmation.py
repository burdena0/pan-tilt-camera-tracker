import numpy as np
from camera_tracker.detection import Detection
from camera_tracker.geometry import Calibration
from camera_tracker.tracking import Tracker,CameraServo

def test_intermittent_false_detections_cannot_accumulate_into_camera_lock():
    calibration=Calibration([[300,0,320],[0,300,240],[0,0,1]],np.zeros(5),(640,480))
    tracker,servo=Tracker(),CameraServo()
    for i in range(30):
        detections=[] if i%3==2 else [Detection(np.array([330,230,350,250]),'drone',.95)]
        tracks=tracker.update(detections,i*.05,calibration,0,0)
        command,_=servo.command(tracks,i*.05,0,0,0)
        assert servo.lock_id is None and not np.any(command)
