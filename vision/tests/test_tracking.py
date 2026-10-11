import numpy as np
import pytest
from camera_tracker.geometry import Calibration,camera_rotation
from camera_tracker.detection import Detection
from camera_tracker.tracking import Tracker,CameraServo

@pytest.fixture
def calibration():return Calibration([[300,0,320],[0,300,240],[0,0,1]],np.zeros(5),(640,480))

@pytest.mark.parametrize('pan,tilt',[(-120,-20),(-40,15),(0,0),(35,45),(120,50)])
def test_center_ray_is_camera_direction(calibration,pan,tilt):
    assert np.allclose(calibration.bearing([320,240],pan,tilt),[pan,tilt],atol=1e-8)
    assert np.allclose(camera_rotation(pan,tilt).T@camera_rotation(pan,tilt),np.eye(3),atol=1e-12)

def detection_at(calibration,bearing,pan,tilt,label='drone'):
    p=calibration.pixel(bearing,pan,tilt)
    return Detection(np.r_[p-10,p+10],label,.95)

def test_camera_rotation_does_not_become_drone_velocity(calibration):
    tracker=Tracker()
    for i in range(20):
        pan,tilt=i*.8,i*.3
        tracks=tracker.update([detection_at(calibration,[15,10],pan,tilt)],i*.05,calibration,pan,tilt)
    assert len(tracks)==1 and tracks[0].confirmed
    assert np.allclose(tracks[0].state,[15,10,0,0],atol=1e-6)

def test_global_assignment_preserves_distinct_crossing_tracks(calibration):
    tracker=Tracker()
    ids=[]
    for i in range(30):
        positions=[[i*.15,4], [4.5-i*.15,-4]]
        detections=[detection_at(calibration,p,0,0) for p in positions]
        if i%2:detections.reverse()
        tracks=tracker.update(detections,i*.05,calibration,0,0)
        ids.append([t.id for t in sorted(tracks,key=lambda t:t.state[1])])
    assert len(tracks)==2 and len(set(tuple(v) for v in ids))==1

def test_missing_track_stops_and_does_not_silently_reassign(calibration):
    tracker,servo=Tracker(),CameraServo()
    for i in range(5):tracks=tracker.update([detection_at(calibration,[4,6],0,0)],i*.05,calibration,0,0)
    velocity,reason=servo.command(tracks,.2,0,0,0)
    assert np.all(velocity>0) and reason=='tracking'
    locked=servo.lock_id
    tracks=tracker.update([], .25,calibration,0,0)
    velocity,reason=servo.command(tracks,.25,0,0,0)
    assert not np.any(velocity) and reason=='occluded / lost' and servo.lock_id==locked
    for i in range(6,30):tracks=tracker.update([detection_at(calibration,[-4,2],0,0)],i*.05,calibration,0,0)
    velocity,_=servo.command(tracks,1.45,0,0,0)
    assert not np.any(velocity) and servo.lock_id==locked

def test_birds_never_acquire_camera_lock(calibration):
    tracker,servo=Tracker(),CameraServo()
    for i in range(5):tracks=tracker.update([detection_at(calibration,[3,5],0,0,'bird')],i*.05,calibration,0,0)
    velocity,_=servo.command(tracks,.2,0,0,0)
    assert servo.lock_id is None and not np.any(velocity)

def test_stale_frame_and_nonmonotonic_time(calibration):
    tracker,servo=Tracker(),CameraServo()
    for i in range(5):tracks=tracker.update([detection_at(calibration,[4,6],0,0)],i*.05,calibration,0,0)
    assert not np.any(servo.command(tracks,.2,.2,0,0)[0])
    with pytest.raises(ValueError):tracker.update([], .2,calibration,0,0)

def test_calibration_rejects_invalid_intrinsics():
    with pytest.raises(ValueError):Calibration(np.eye(3)*np.nan,np.zeros(5),(640,480))

def test_accelerating_night_observations_keep_one_track(calibration):
    tracker=Tracker();ids=[]
    for i in range(160):
        t=i*.05
        bearing=[14*np.sin(1.8*t),10+4*np.sin(2.4*t)]
        tracks=tracker.update([detection_at(calibration,bearing,0,0)],t,calibration,0,0,mode='night')
        ids.append(tracks[0].id)
        assert len(tracks)==1
    assert len(set(ids))==1
