import argparse
import hashlib
import json
import time
import cv2
import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader,TensorDataset
from .crops import crop_rgb,normalize_contrast
from .simulation import ROOT

class CropNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.features=nn.Sequential(nn.Conv2d(3,8,3,padding=1),nn.ReLU(),nn.MaxPool2d(2),
            nn.Conv2d(8,16,3,padding=1),nn.ReLU(),nn.MaxPool2d(2),
            nn.Conv2d(16,24,3,padding=1),nn.ReLU(),nn.MaxPool2d(2))
        self.head=nn.Sequential(nn.Flatten(),nn.Linear(24*8*8,32),nn.ReLU(),nn.Linear(32,3))
    def forward(self,x):return self.head(self.features(x))

def crops(images,masks,seed):
    rng=np.random.default_rng(seed);xs=[];ys=[]
    for rgb,mask in zip(images,masks):
        bgr=cv2.cvtColor(rgb.transpose(1,2,0),cv2.COLOR_RGB2BGR)
        for label in (1,2):
            closed=cv2.morphologyEx((mask==label).astype(np.uint8),cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
            count,_,stats,_=cv2.connectedComponentsWithStats(closed)
            for x,y,w,h,area in stats[1:count]:
                if area<12 or w<5 or h<3:continue
                # vary proposal alignment; perfect training boxes aren't available at runtime.
                jitter=rng.uniform(-.08,.08,4)*max(w,h)
                box=np.array([x,y,x+w,y+h])+jitter
                xs.append(crop_rgb(bgr,box));ys.append(label)
        for _ in range(10):
            w=int(rng.integers(10,45));h=int(rng.integers(5,25))
            x=int(rng.integers(0,320-w));y=int(rng.integers(0,192-h))
            if np.any(mask[max(0,y-h):min(192,y+2*h),max(0,x-w):min(320,x+2*w)]):continue
            xs.append(crop_rgb(bgr,[x,y,x+w,y+h]));ys.append(0);break
    return np.stack(xs),np.array(ys,np.int64)

def evaluate(model,loader,threshold=.9):
    confusion=np.zeros((3,3),np.int64);accepted=np.zeros((3,3),np.int64);model.eval()
    with torch.no_grad():
        for x,y in loader:
            confidence=model(x.float()/255).softmax(1).numpy();p=confidence.argmax(1)
            confusion+=np.bincount(y.numpy()*3+p,minlength=9).reshape(3,3)
            keep=confidence.max(1)>=threshold
            accepted+=np.bincount(y.numpy()[keep]*3+p[keep],minlength=9).reshape(3,3)
    return {'accuracy':float(np.trace(confusion)/confusion.sum()),'confusion':confusion.tolist(),
            'confidence_threshold':threshold,'accepted_confusion':accepted.tolist(),
            'accepted_drone_precision':float(accepted[1,1]/max(accepted[:,1].sum(),1)),
            'accepted_drone_recall':float(accepted[1,1]/max(confusion[1].sum(),1)),
            'accepted_fraction':float(accepted.sum()/confusion.sum())}

def bundle():
    main=ROOT/'vision/models/synthetic_drone.json'
    verifier=ROOT/'vision/models/crop_classifier.json'
    manifest=json.loads(main.read_text())
    manifest['verifier']={'file':'crop_classifier.onnx',**json.loads(verifier.read_text())}
    main.write_text(json.dumps(manifest,indent=2))

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--epochs',type=int,default=24)
    parser.add_argument('--bundle-only',action='store_true')
    parser.add_argument('--dataset',type=str)
    parser.add_argument('--name',choices=['crop_classifier','crop_night'],default='crop_classifier')
    parser.add_argument('--normalize-contrast',action='store_true')
    parser.add_argument('--prepared-crops',action='store_true')
    args=parser.parse_args()
    if args.bundle_only:bundle();return
    torch.set_num_threads(4);torch.manual_seed(31415)
    cache=ROOT/args.dataset if args.dataset else ROOT/'vision/output/dataset_lighting.npz'
    if not cache.exists():cache=ROOT/'vision/output/dataset_ray_checked.npz'
    data=np.load(cache)
    pairs=[(data[f'{name}_x'],data[f'{name}_y']) if args.prepared_crops else crops(data[f'{name}_x'],data[f'{name}_y'],seed)
           for name,seed in [('train',10),('val',20),('test',30)]]
    if args.normalize_contrast and not args.prepared_crops:
        pairs=[(np.stack([normalize_contrast(crop) for crop in x]),y) for x,y in pairs]
    loaders=[DataLoader(TensorDataset(torch.from_numpy(x),torch.from_numpy(y)),batch_size=64,shuffle=i==0)
             for i,(x,y) in enumerate(pairs)]
    model=CropNet();optimizer=torch.optim.AdamW(model.parameters(),lr=.001)
    counts=np.bincount(pairs[0][1],minlength=3);weights=torch.tensor(counts.sum()/counts/3,dtype=torch.float32)
    output=ROOT/'vision/models';best=-1;history=[];started=time.perf_counter()
    for epoch in range(args.epochs):
        model.train();total=0
        for x,y in loaders[0]:
            loss=nn.functional.cross_entropy(model(x.float()/255),y,weight=weights)
            optimizer.zero_grad();loss.backward();optimizer.step();total+=float(loss.detach())
        val=evaluate(model,loaders[1]);history.append({'epoch':epoch+1,'loss':total/len(loaders[0]),'validation':val})
        if val['accuracy']>best:best=val['accuracy'];torch.save(model.state_dict(),output/f'{args.name}.pt')
        print(f'crop epoch {epoch+1}/{args.epochs} val_accuracy={val["accuracy"]:.4f}',flush=True)
    model.load_state_dict(torch.load(output/f'{args.name}.pt',weights_only=True));model.eval()
    path=output/f'{args.name}.onnx'
    torch.onnx.export(model,torch.zeros(1,3,64,64),str(path),opset_version=13,dynamo=False,
                      input_names=['rgb'],output_names=['logits'],dynamic_axes={'rgb':{0:'batch'},'logits':{0:'batch'}})
    manifest={'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'input_size':[64,64],
              'labels':['background','drone','bird'],'preprocess':'RGB float32 CHW [0,1], 1.35x square crop',
              'domain':'synthetic only; real drone accuracy unverified','parameters':sum(p.numel() for p in model.parameters()),
              'normalize_contrast':args.normalize_contrast}
    path.with_suffix('.json').write_text(json.dumps(manifest,indent=2))
    report={'seed':31415,'split_crops':dict(zip(['train','validation','test'],[len(x) for x,y in pairs])),
            'train_class_counts':counts.tolist(),'parameters':manifest['parameters'],'history':history,
            'test':evaluate(model,loaders[2]),'training_seconds':time.perf_counter()-started,
            'domain':manifest['domain'],'evaluation':'ground-truth-derived jittered crops; pipeline performance reported separately'}
    report['cached_dataset_sha256']=hashlib.sha256(cache.read_bytes()).hexdigest()
    report['normalize_contrast']=args.normalize_contrast
    report['prepared_crops']=args.prepared_crops
    if args.prepared_crops:report['evaluation']='actual detector proposals plus jittered geometric crops; source scene splits remain disjoint'
    report_name='crop_night_training' if args.name=='crop_night' else 'crop_training'
    (ROOT/f'reports/{report_name}.json').write_text(json.dumps(report,indent=2))
    if args.name=='crop_classifier':bundle()
    else:
        night=ROOT/'vision/models/night_segmentation.json';metadata=json.loads(night.read_text())
        metadata['verifier']={'file':path.name,**manifest};night.write_text(json.dumps(metadata,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='history'},indent=2))
if __name__=='__main__':main()
