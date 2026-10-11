"""train on the boxes the night pipeline actually proposes, including bad boxes."""
import hashlib,json
from pathlib import Path
import cv2,numpy as np
from camera_tracker.detection import NeuralDetector
from camera_tracker.crops import crop_rgb,normalize_contrast
from camera_tracker.train_classifier import crops
from camera_tracker.simulation import ROOT

source=ROOT/'vision/output/dataset_lighting.npz';data=np.load(source)
detector=NeuralDetector(ROOT/'vision/models/night_segmentation.onnx')
detector.contrast_kernel=5
output={};counts={}
for split,seed in [('train',10),('val',20),('test',30)]:
    xs,ys=crops(data[f'{split}_x'],data[f'{split}_y'],seed)
    xs=list(xs);ys=list(ys)
    for i,(rgb,mask) in enumerate(zip(data[f'{split}_x'],data[f'{split}_y'])):
        bgr=cv2.cvtColor(rgb.transpose(1,2,0),cv2.COLOR_RGB2BGR)
        boxes=[]
        for label in (1,2):
            _,_,stats,_=cv2.connectedComponentsWithStats(cv2.morphologyEx((mask==label).astype(np.uint8),cv2.MORPH_CLOSE,np.ones((3,3),np.uint8)))
            boxes.extend((np.array([x,y,x+w,y+h]),label) for x,y,w,h,area in stats[1:] if area>=12)
        proposals,_=detector.detect(bgr,verify=False)
        for d in proposals:
            scores=[]
            for box,label in boxes:
                intersection=np.maximum(0,np.minimum(d.bbox[2:],box[2:])-np.maximum(d.bbox[:2],box[:2])).prod()
                union=np.prod(d.bbox[2:]-d.bbox[:2])+np.prod(box[2:]-box[:2])-intersection
                scores.append((intersection/max(union,1),label))
            score,label=max(scores,default=(0,0))
            # exclude ambiguous partial crops instead of teaching contradictory labels.
            if .1<score<.4:continue
            xs.append(crop_rgb(bgr,d.bbox));ys.append(label if score>=.4 else 0)
        if (i+1)%400==0:print(split,i+1,flush=True)
    output[f'{split}_x']=np.stack([normalize_contrast(c) for c in xs]);output[f'{split}_y']=np.array(ys,np.int64)
    counts[split]=np.bincount(ys,minlength=3).tolist()
path=ROOT/'vision/output/proposal_crops.npz';np.savez_compressed(path,**output)
(ROOT/'reports/proposal_dataset.json').write_text(json.dumps({'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
    'segmentation_sha256':detector.manifest['sha256'],'contrast_close_kernel':5,
    'output_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'split_class_counts':counts,
    'label_rule':'IoU >= .4 against held-out geometric object boxes; IoU <= .1 is background; ambiguous boxes excluded',
    'preprocessing':'shared RGB percentile stretch and Gaussian smoothing','leakage':'source scene splits remain separate'},indent=2))
print(counts)
