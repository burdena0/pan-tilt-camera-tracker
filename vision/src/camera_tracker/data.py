"""load real image/mask pairs without leaking the same image across splits."""
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np

def load_manifest(path,size=(320,192)):
    path=Path(path).resolve();root=path.parent
    data=json.loads(path.read_text())
    if data.get('labels')!=['background','drone','bird']:raise ValueError('label order must be background, drone, bird')
    seen=set();splits=[]
    for split in ('train','validation','test'):
        images,masks=[],[]
        for item in data.get(split,[]):
            image_path=(root/item['image']).resolve();mask_path=(root/item['mask']).resolve()
            if not image_path.is_relative_to(root) or not mask_path.is_relative_to(root):raise ValueError('dataset paths must stay inside the manifest folder')
            image=cv2.imread(str(image_path));mask=cv2.imread(str(mask_path),cv2.IMREAD_UNCHANGED)
            if image is None or mask is None or mask.ndim!=2 or image.shape[:2]!=mask.shape:raise ValueError('invalid image/mask pair')
            if not set(np.unique(mask)).issubset({0,1,2}):raise ValueError('masks must contain only label IDs 0, 1 and 2')
            digest=hashlib.sha256(image.tobytes()).hexdigest()
            if digest in seen:raise ValueError('duplicate image across or within splits')
            seen.add(digest)
            images.append(cv2.cvtColor(cv2.resize(image,size),cv2.COLOR_BGR2RGB).transpose(2,0,1))
            masks.append(cv2.resize(mask,size,interpolation=cv2.INTER_NEAREST))
        if not images:raise ValueError(f'empty {split} split')
        splits.extend([np.stack(images),np.stack(masks)])
    return splits
