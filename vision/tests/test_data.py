import json
import cv2
import numpy as np
import pytest
from camera_tracker.data import load_manifest

def dataset(tmp_path):
    data={'labels':['background','drone','bird']}
    for i,split in enumerate(('train','validation','test')):
        image=np.full((16,24,3),i*40,np.uint8);mask=np.zeros((16,24),np.uint8);mask[4:8,4:8]=1
        cv2.imwrite(str(tmp_path/f'{split}.png'),image);cv2.imwrite(str(tmp_path/f'{split}_mask.png'),mask)
        data[split]=[{'image':f'{split}.png','mask':f'{split}_mask.png'}]
    path=tmp_path/'manifest.json';path.write_text(json.dumps(data));return path,data

def test_real_dataset_loader_shape_and_labels(tmp_path):
    path,_=dataset(tmp_path);arrays=load_manifest(path)
    assert arrays[0].shape==(1,3,192,320) and arrays[1].shape==(1,192,320)
    assert set(np.unique(arrays[1]))=={0,1}

def test_duplicate_images_are_rejected(tmp_path):
    path,data=dataset(tmp_path);data['test'][0]['image']='train.png';path.write_text(json.dumps(data))
    with pytest.raises(ValueError,match='duplicate'):load_manifest(path)

def test_manifest_escape_is_rejected(tmp_path):
    path,data=dataset(tmp_path);data['test'][0]['image']='../outside.png';path.write_text(json.dumps(data))
    with pytest.raises(ValueError,match='inside'):load_manifest(path)

def test_bad_mask_labels_are_rejected(tmp_path):
    path,_=dataset(tmp_path);cv2.imwrite(str(tmp_path/'test_mask.png'),np.full((16,24),255,np.uint8))
    with pytest.raises(ValueError,match='IDs'):load_manifest(path)
