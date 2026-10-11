import argparse
import hashlib
import json
from pathlib import Path
import time
import cv2
import numpy as np
import torch
from torch.nn import functional as F
from torch.utils.data import TensorDataset, DataLoader
from .model import SegmentationNet
from .simulation import Scene, ROOT

def dataset(count, seed, hard=False, low_light=False):
    rng = np.random.default_rng(seed)
    scene = Scene(low_light=low_light)
    images, masks = [], []
    base_size, base_pos = scene.model.geom_size.copy(), scene.model.geom_pos.copy()
    base_radius = scene.model.geom_rbound.copy()
    for i in range(count):
        pan, tilt = rng.uniform(-15,15), rng.uniform(0,25)
        scene.set_angles(pan,tilt)
        if low_light:
            scene.spotlight(rng.random()<.65)
            scene.model.light_diffuse[scene.model.light('tracking_spotlight').id]=rng.uniform(2,6)*np.array([1,.95,.875])
        for name in ('drone0','drone1','bird'):
            enabled = rng.random() < (.65 if name=='drone0' else .35)
            body = scene.model.body(name).id
            selected = scene.model.geom_bodyid == body
            scale = rng.uniform(.7,1.6)
            scene.model.geom_size[selected] = base_size[selected] * scale
            scene.model.geom_pos[selected] = base_pos[selected] * scale
            scene.model.geom_rbound[selected] = base_radius[selected] * scale
            tone = rng.uniform(.025,.4)
            scene.model.geom_rgba[selected,:3] = tone + rng.uniform(0,.04,3)
            distance = rng.uniform(2.0,6.5)
            # sample positions relative to the virtual camera, not labels from a shortcut.
            from .geometry import camera_rotation
            ray = camera_rotation(pan,tilt) @ np.array([rng.uniform(-.48,.48),rng.uniform(-.28,.28),1])
            position = ray * distance + scene.data.cam_xpos[scene.model.camera('sensor').id]
            scene.pose(name, position if enabled else [0,0,-10], rng.uniform([-.6,-.6,-np.pi],[.6,.6,np.pi]))
        bgr, mask, _ = scene.render(annotation=True)
        gain = rng.uniform(.6,1.3) if not hard else rng.uniform(.45,1.4)
        bgr = bgr.astype(np.float32)*gain + rng.uniform(-15,15)
        bgr += rng.normal(0, rng.uniform(0,5 if not hard else 9), bgr.shape)
        bgr = np.clip(bgr,0,255).astype(np.uint8)
        if rng.random() < (.2 if not hard else .55): bgr = cv2.GaussianBlur(bgr,(3,3),rng.uniform(.4,1.0))
        images.append(cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB).transpose(2,0,1))
        masks.append(mask)
        if (i+1)%200==0: print(f'generated {i+1}/{count} seed={seed}',flush=True)
    scene.close()
    return np.stack(images), np.stack(masks)

def scores(model, loader):
    confusion = np.zeros((3,3),np.int64)
    model.eval()
    with torch.no_grad():
        for x,y in loader:
            predicted = model(x.float()/255).argmax(1).numpy()
            confusion += np.bincount((y.numpy().ravel()*3+predicted.ravel()),minlength=9).reshape(3,3)
    intersection = np.diag(confusion)
    iou = intersection / np.maximum(confusion.sum(0)+confusion.sum(1)-intersection,1)
    return {'class_iou': dict(zip(['background','drone','bird'],iou.tolist())),
            'foreground_macro_iou':float(iou[1:].mean()),'confusion_pixels':confusion.tolist()}

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--epochs',type=int,default=20)
    parser.add_argument('--train-count',type=int,default=1600)
    parser.add_argument('--data-manifest',type=Path)
    parser.add_argument('--resume',type=Path)
    parser.add_argument('--learning-rate',type=float,default=.002)
    parser.add_argument('--lighting-curriculum',action='store_true')
    args=parser.parse_args()
    if args.epochs<1 or args.train_count<1 or not np.isfinite(args.learning_rate) or not 0<args.learning_rate<=1:
        parser.error('epochs, image count and learning rate must be positive')
    torch.set_num_threads(4); torch.manual_seed(1729); np.random.seed(1729)
    output=ROOT/'vision/models'; output.mkdir(parents=True,exist_ok=True)
    cache=ROOT/'vision/output'/('dataset_lighting.npz' if args.lighting_curriculum else 'dataset_ray_checked.npz')
    cache.parent.mkdir(parents=True,exist_ok=True)
    if args.data_manifest:
        from .data import load_manifest
        train_x,train_y,val_x,val_y,test_x,test_y=load_manifest(args.data_manifest)
    elif cache.exists():
        data=np.load(cache)
        train_x,train_y,val_x,val_y,test_x,test_y=[data[k] for k in ('train_x','train_y','val_x','val_y','test_x','test_y')]
        if len(train_x)!=args.train_count: raise ValueError('cached training size differs; use a separate cache')
    elif args.lighting_curriculum:
        base=np.load(ROOT/'vision/output/dataset_ray_checked.npz')
        night=[dataset(count,seed,hard=True,low_light=True) for count,seed in [(600,120000),(100,140000),(200,190000)]]
        train_x=np.concatenate([base['train_x'][:1000],night[0][0]]);train_y=np.concatenate([base['train_y'][:1000],night[0][1]])
        val_x=np.concatenate([base['val_x'],night[1][0]]);val_y=np.concatenate([base['val_y'],night[1][1]])
        test_x=np.concatenate([base['test_x'],night[2][0]]);test_y=np.concatenate([base['test_y'],night[2][1]])
        np.savez_compressed(cache,train_x=train_x,train_y=train_y,val_x=val_x,val_y=val_y,test_x=test_x,test_y=test_y)
    else:
        train_x,train_y=dataset(args.train_count,20000,hard=True)
        val_x,val_y=dataset(200,40000)
        test_x,test_y=dataset(400,90000,hard=True)
        np.savez_compressed(cache,train_x=train_x,train_y=train_y,val_x=val_x,val_y=val_y,test_x=test_x,test_y=test_y)
    loaders=[DataLoader(TensorDataset(torch.from_numpy(x),torch.from_numpy(y).long()),batch_size=24,shuffle=i==0)
             for i,(x,y) in enumerate(((train_x,train_y),(val_x,val_y),(test_x,test_y)))]
    model=SegmentationNet()
    resume_hash = None
    if args.resume:
        resume_hash = hashlib.sha256(args.resume.read_bytes()).hexdigest()
        model.load_state_dict(torch.load(args.resume,weights_only=True))
    optimizer=torch.optim.AdamW(model.parameters(),lr=args.learning_rate,weight_decay=1e-4)
    best=-1.; history=[]; started=time.perf_counter()
    for epoch in range(args.epochs):
        model.train(); total=0
        for x,y in loaders[0]:
            logits=model(x.float()/255)
            ce=F.cross_entropy(logits,y,weight=torch.tensor([.1,1.,1.]))
            probability=logits.softmax(1)
            truth=F.one_hot(y,3).permute(0,3,1,2).float()
            dice=1-(2*(probability[:,1:]*truth[:,1:]).sum((0,2,3))+1)/(probability[:,1:].sum((0,2,3))+truth[:,1:].sum((0,2,3))+1)
            loss=ce+.5*dice.mean()
            optimizer.zero_grad();loss.backward();optimizer.step();total+=float(loss.detach())
        validation=scores(model,loaders[1]); metric=validation['foreground_macro_iou']
        history.append({'epoch':epoch+1,'loss':total/len(loaders[0]),'validation':validation})
        if metric>best:
            best=metric;torch.save(model.state_dict(),output/'synthetic_drone.pt')
        print(f'epoch {epoch+1}/{args.epochs} loss={total/len(loaders[0]):.4f} val_iou={metric:.4f}',flush=True)
    model.load_state_dict(torch.load(output/'synthetic_drone.pt',weights_only=True));model.eval()
    report={'seed':1729,'split_seeds':{'train':20000,'validation':40000,'test':90000},
            'train_images':len(train_x),'validation_images':len(val_x),'test_images':len(test_x),
            'data_source':str(args.data_manifest) if args.data_manifest else 'randomized MuJoCo-rendered synthetic drone and bird geometry',
            'domain':'user-provided labeled image/mask splits' if args.data_manifest else 'synthetic only; real drone accuracy unverified',
            'parameters':sum(p.numel() for p in model.parameters()),'history':history,
            'epochs_this_run':args.epochs,'learning_rate':args.learning_rate,'resume_weights_sha256':resume_hash,
            'annotation_method':'renderer candidates verified against nearest geometric ray intersections',
            'cached_dataset_sha256':None if args.data_manifest else hashlib.sha256(cache.read_bytes()).hexdigest(),
            'lighting_curriculum':args.lighting_curriculum,
            'night_split_seeds':{'train':120000,'validation':140000,'test':190000} if args.lighting_curriculum else None,
            'test':scores(model,loaders[2]),'training_seconds':time.perf_counter()-started}
    path=output/'synthetic_drone.onnx'
    torch.onnx.export(model,torch.zeros(1,3,192,320),str(path),opset_version=13,
                      input_names=['rgb'],output_names=['logits'],dynamo=False)
    manifest={'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'input_size':[320,192],
              'labels':['background','drone','bird'],'preprocess':'RGB float32 CHW [0,1]',
              'domain':report['domain'],
              'parameters':report['parameters']}
    path.with_suffix('.json').write_text(json.dumps(manifest,indent=2))
    reports=ROOT/'reports';reports.mkdir(exist_ok=True)
    (reports/'vision_training.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='history'},indent=2))

if __name__=='__main__': main()
