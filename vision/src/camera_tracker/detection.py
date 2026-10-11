from dataclasses import dataclass
from collections import deque
import hashlib
import json
from pathlib import Path
import time
import cv2
import numpy as np
import onnxruntime as ort
from .crops import crop_rgb,normalize_contrast
from .optical_flow import TemporalProposals

@dataclass
class Detection:
    bbox: np.ndarray
    label: str
    score: float
    source: str = 'neural'
    @property
    def center(self): return (self.bbox[:2] + self.bbox[2:]) / 2

class NeuralDetector:
    def __init__(self, model_path, threshold=.55, threads=4):
        path = Path(model_path)
        self.manifest = json.loads(path.with_suffix('.json').read_text())
        if hashlib.sha256(path.read_bytes()).hexdigest() != self.manifest['sha256']:
            raise ValueError("model hash mismatch")
        self.size = tuple(self.manifest['input_size'])
        self.labels = self.manifest['labels']
        self.use_contrast=self.manifest.get('contrast_proposals',False)
        self.contrast_kernel=self.manifest.get('contrast_close_kernel',7)
        if not isinstance(self.contrast_kernel,int) or self.contrast_kernel not in range(1,16,2):raise ValueError('invalid contrast kernel')
        self.verification_threshold=self.manifest.get('verification_threshold',.9)
        self.temporal=TemporalProposals() if self.manifest.get('temporal_proposals') else None
        self.frame_number=0
        self.threshold,self.timings = threshold,deque(maxlen=10000)
        options = ort.SessionOptions(); options.intra_op_num_threads = threads
        self.session = ort.InferenceSession(str(path), sess_options=options, providers=['CPUExecutionProvider'])
        self.input = self.session.get_inputs()[0].name
        if self.session.get_inputs()[0].shape != [1, 3, self.size[1], self.size[0]]:
            raise ValueError("model input shape doesn't match manifest")
        self.verifier = None
        if self.manifest.get('verifier'):
            manifest=self.manifest['verifier']
            if manifest['labels']!=self.labels or manifest['input_size']!=[64,64]:raise ValueError('incompatible crop verifier')
            verify_path=path.parent/manifest['file']
            if verify_path.resolve().parent!=path.resolve().parent or hashlib.sha256(verify_path.read_bytes()).hexdigest()!=manifest['sha256']:
                raise ValueError('crop verifier path/hash mismatch')
            self.verifier=ort.InferenceSession(str(verify_path),sess_options=options,providers=['CPUExecutionProvider'])
        self.night=None;self.low_light=False;self.active_mode='daylight'
        if self.manifest.get('illumination_expert'):
            expert=self.manifest['illumination_expert'];expert_path=path.parent/expert['file']
            if expert_path.resolve().parent!=path.resolve().parent or expert_path.resolve()==path.resolve() or hashlib.sha256(expert_path.read_bytes()).hexdigest()!=expert['sha256']:
                raise ValueError('illumination expert path/hash mismatch')
            self.night=NeuralDetector(expert_path,threshold,threads)

    def detect(self, bgr, verify=True):
        if self.night is not None:
            was_low_light=self.low_light
            brightness=float(cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY).mean())
            expert=self.manifest['illumination_expert']
            if brightness<expert['enter_brightness']:self.low_light=True
            if brightness>expert['exit_brightness']:self.low_light=False
            if was_low_light and not self.low_light:self.night.reset_temporal()
            self.active_mode='night' if self.low_light else 'daylight'
            if self.low_light:
                result=self.night.detect(bgr,verify=verify);self.timings.append(self.night.timings[-1]);return result
        height, width = bgr.shape[:2]
        start = time.perf_counter()
        resized = cv2.resize(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), self.size)
        tensor = np.ascontiguousarray(resized.transpose(2, 0, 1)[None], dtype=np.float32) / 255
        logits = self.session.run(None, {self.input: tensor})[0][0]
        if logits.shape != (len(self.labels), self.size[1], self.size[0]) or not np.isfinite(logits).all():
            raise ValueError("invalid model output")
        probability = np.exp(logits - logits.max(axis=0, keepdims=True))
        probability /= probability.sum(axis=0, keepdims=True)
        classes = probability.argmax(axis=0)
        detections = []
        raw = ((classes != 0) & (probability[1:].max(0) >= self.threshold)).astype(np.uint8)
        mask = cv2.morphologyEx(raw, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        count,components,stats,_ = cv2.connectedComponentsWithStats(mask)
        for component,(x,y,w,h,area) in enumerate(stats[1:count],1):
            if area < 12 or w < 5 or h < 3: continue
            if x == 0 or y == 0 or x+w >= self.size[0] or y+h >= self.size[1]: continue
            selected = (components[y:y+h,x:x+w] == component) & raw[y:y+h,x:x+w].astype(bool)
            if not selected.any(): continue
            votes = probability[1:,y:y+h,x:x+w][:,selected].sum(1)
            label_id = int(votes.argmax())+1
            # classify the whole silhouette; mixed bird/drone fragments don't get two IDs.
            if self.verifier is None and votes[label_id-1] / max(float(votes.sum()),1e-9) < .7: continue
            score = float(probability[label_id,y:y+h,x:x+w][selected].mean())
            bbox = np.array([x*width/self.size[0],y*height/self.size[1],(x+w)*width/self.size[0],(y+h)*height/self.size[1]])
            detections.append(Detection(bbox,self.labels[label_id],score))
        if self.verifier is not None and self.use_contrast:
            gray=cv2.cvtColor(resized,cv2.COLOR_RGB2GRAY)
            smooth=cv2.GaussianBlur(gray,(3,3),.7)
            background=cv2.GaussianBlur(smooth,(31,31),6)
            noise=float(np.median(np.abs(gray.astype(float)-smooth))*1.4826)
            residual=cv2.absdiff(smooth,background)
            proposals=(residual>=max(10,noise*3)).astype(np.uint8)
            proposals=cv2.morphologyEx(proposals,cv2.MORPH_CLOSE,np.ones((self.contrast_kernel,self.contrast_kernel),np.uint8))
            count,_,stats,_=cv2.connectedComponentsWithStats(proposals)
            for x,y,w,h,area in stats[1:count]:
                if area<12 or area>1200 or w<5 or h<3 or w>100 or h>80:continue
                if x<2 or y<2 or x+w>=self.size[0]-2 or y+h>=self.size[1]-2:continue
                box=np.array([x,y,x+w,y+h])*np.array([width/self.size[0],height/self.size[1]]*2)
                overlaps=[d for d in detections if np.all(np.minimum(d.bbox[2:],box[2:])-np.maximum(d.bbox[:2],box[:2])>0)]
                for d in overlaps:
                    box=np.r_[np.minimum(box[:2],d.bbox[:2]),np.maximum(box[2:],d.bbox[2:])]
                detections=[d for d in detections if all(d is not old for old in overlaps)]
                detections.append(Detection(box,'background',0.))
        if self.temporal is not None and verify:
            proposals=self.temporal.propose(bgr)
            self.frame_number+=1
            # periodically reseed from fresh neural geometry instead of drifting indefinitely.
            if self.frame_number%20:
                for box,label in proposals:
                    overlaps=[]
                    for d in detections:
                        intersection=np.maximum(0,np.minimum(box[2:],d.bbox[2:])-np.maximum(box[:2],d.bbox[:2])).prod()
                        union=np.prod(box[2:]-box[:2])+np.prod(d.bbox[2:]-d.bbox[:2])-intersection
                        if intersection/max(union,1)>.2:overlaps.append(d)
                    detections=[d for d in detections if all(d is not old for old in overlaps)]
                    detections.append(Detection(box,label,0.,'optical_flow'))
        if verify and self.verifier is not None and detections:
            transform=normalize_contrast if self.manifest['verifier'].get('normalize_contrast') else lambda x:x
            crops=np.stack([transform(crop_rgb(bgr,d.bbox)) for d in detections]).astype(np.float32)/255
            logits=self.verifier.run(None,{self.verifier.get_inputs()[0].name:crops})[0]
            if logits.shape!=(len(detections),3) or not np.isfinite(logits).all():raise ValueError('invalid crop verifier output')
            confidence=np.exp(logits-logits.max(1,keepdims=True));confidence/=confidence.sum(1,keepdims=True)
            verified=[]
            for d,p in zip(detections,confidence):
                label=int(p.argmax())
                threshold=.9 if d.source=='optical_flow' else self.verification_threshold
                if label==0 or p[label]<threshold:continue
                if d.source=='optical_flow' and self.labels[label]!=d.label:continue
                verified.append(Detection(d.bbox,self.labels[label],float(p[label]),d.source))
            detections=verified
        if self.temporal is not None and verify:self.temporal.commit(detections)
        self.timings.append((time.perf_counter() - start) * 1000)
        return detections, probability

    def reset_temporal(self):
        if self.temporal is not None:self.temporal=TemporalProposals()
        self.frame_number=0
        if self.night is not None:self.night.reset_temporal()
