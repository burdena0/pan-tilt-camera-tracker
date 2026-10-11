import numpy as np
import cv2
from camera_tracker.simulation import Scene

def test_occluder_hides_target_labels_and_removed_occluder_reveals_them():
    scene=Scene()
    try:
        scene.set_angles(12,10)
        scene.pose('drone0',[2.8,.3,.55])
        scene.pose('drone1',[0,0,-10]);scene.pose('bird',[0,0,-10])
        _,labels,instances=scene.render(True)
        assert np.count_nonzero(instances==1)>20
        scene.pose('occluder',[1.4,.15,.355])
        _,labels,instances=scene.render(True)
        assert not np.any(instances) and not np.any(labels)
    finally:scene.close()

def test_shorter_shutter_preserves_moving_object_edges():
    scene=Scene()
    try:
        scene.set_angles(0,10);scene.pose('drone1',[0,0,-10]);scene.pose('bird',[0,0,-10])
        def trajectory(t):scene.pose('drone0',[2.8,2*t,.64])
        trajectory(.2)
        short=scene.render_exposure(trajectory,.2,2)
        long=scene.render_exposure(trajectory,.2,40)
        energy=lambda image:cv2.Laplacian(cv2.cvtColor(image[75:115,100:220],cv2.COLOR_BGR2GRAY),cv2.CV_64F).var()
        assert energy(short)>energy(long)*1.3
    finally:scene.close()

def test_spotlight_changes_target_pixels_in_low_light():
    scene=Scene(low_light=True)
    try:
        scene.set_angles(0,10);scene.pose('drone0',[2.8,0,.64])
        off,_,instances=scene.render(True)
        scene.spotlight(True);on=scene.render()
        assert (on[instances==1].astype(float)-off[instances==1]).mean()>5
    finally:scene.close()
