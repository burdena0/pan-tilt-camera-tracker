"""preserve the night specialist, then bundle it with the daylight checkpoints."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import torch
from camera_tracker.model import SegmentationNet
root=Path(__file__).resolve().parents[1];models=root/'vision/models';reports=root/'reports'
parser=argparse.ArgumentParser();parser.add_argument('--restore-daylight',action='store_true');args=parser.parse_args()
if args.restore_daylight:
    current=json.loads((reports/'vision_training.json').read_text())
    if not current.get('lighting_curriculum'):raise RuntimeError('only preserve a recorded lighting adaptation')
    for source,dest in [('synthetic_drone','night_segmentation'),('crop_classifier','crop_night')]:
        for suffix in ('.onnx','.json','.pt'):shutil.copyfile(models/(source+suffix),models/(dest+suffix))
    night=json.loads((models/'night_segmentation.json').read_text())
    night['verifier']['file']='crop_night.onnx'
    (models/'night_segmentation.json').write_text(json.dumps(night,indent=2))
    shutil.copyfile(reports/'vision_training.json',reports/'night_training.json')
    shutil.copyfile(reports/'crop_training.json',reports/'crop_night_training.json')
    shutil.copyfile(reports/'vision_simulation.json',reports/'illumination_regression.json')
    shutil.copyfile(models/'bootstrap_synthetic.pt',models/'synthetic_drone.pt')
    model=SegmentationNet();model.load_state_dict(torch.load(models/'synthetic_drone.pt',weights_only=True));model.eval()
    path=models/'synthetic_drone.onnx'
    torch.onnx.export(model,torch.zeros(1,3,192,320),str(path),opset_version=13,dynamo=False,input_names=['rgb'],output_names=['logits'])
    manifest={'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'input_size':[320,192],
              'labels':['background','drone','bird'],'preprocess':'RGB float32 CHW [0,1]',
              'domain':'daylight synthetic simulator geometry; real accuracy unverified','parameters':23427}
    path.with_suffix('.json').write_text(json.dumps(manifest,indent=2))
    shutil.copyfile(reports/'daylight_training.json',reports/'vision_training.json')
else:
    night_manifest=models/'night_segmentation.json'
    night=json.loads(night_manifest.read_text());night['contrast_proposals']=True
    night['temporal_proposals']=True
    night['contrast_close_kernel']=7
    night['verification_threshold']=.75
    night_manifest.write_text(json.dumps(night,indent=2))
    path=models/'synthetic_drone.json';manifest=json.loads(path.read_text())
    night_path=models/'night_segmentation.onnx'
    manifest['illumination_expert']={'file':night_path.name,'sha256':hashlib.sha256(night_path.read_bytes()).hexdigest(),
                                   'enter_brightness':50,'exit_brightness':80}
    path.write_text(json.dumps(manifest,indent=2))
