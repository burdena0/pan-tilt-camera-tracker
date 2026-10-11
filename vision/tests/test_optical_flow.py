import cv2,numpy as np
from camera_tracker.optical_flow import TemporalProposals
from camera_tracker.detection import Detection

def test_forward_backward_features_follow_translation():
    rng=np.random.default_rng(78)
    image=np.full((192,320,3),20,np.uint8)
    image[80:100,145:175]=rng.integers(40,230,(20,30,3),dtype=np.uint8)
    proposer=TemporalProposals()
    assert proposer.propose(image)==[]
    box=np.array([145,80,175,100],float)
    proposer.commit([Detection(box,'drone',.99)])
    moved=cv2.warpAffine(image,np.array([[1,0,4],[0,1,3]],np.float32),(320,192),borderValue=(20,20,20))
    proposals=proposer.propose(moved)
    assert len(proposals)==1 and proposals[0][1]=='drone'
    assert np.allclose(proposals[0][0],box+[4,3,4,3],atol=.2)

def test_textureless_image_cannot_supply_a_flow_proposal():
    proposer=TemporalProposals();blank=np.full((192,320,3),20,np.uint8)
    proposer.propose(blank);proposer.commit([Detection(np.array([140,80,170,100]),'drone',.99)])
    assert proposer.propose(blank)==[]
