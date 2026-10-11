"""check exported weights against the trained torch checkpoints."""
from pathlib import Path
import hashlib
import json
import numpy as np
import onnx
import onnxruntime as ort
import torch
from torch.utils.data import DataLoader,TensorDataset
from camera_tracker.model import SegmentationNet
from camera_tracker.train_classifier import CropNet,crops,evaluate
from camera_tracker.crops import normalize_contrast

root=Path(__file__).resolve().parents[1]
torch.set_num_threads(4)
errors={}
for name,dataset in [('synthetic_drone','dataset_ray_checked'),('crop_classifier','dataset_ray_checked'),
                     ('night_segmentation','dataset_lighting'),('crop_night','proposal_crops')]:
    data=np.load(root/f'vision/output/{dataset}.npz')
    is_crop=name.startswith('crop_')
    model=CropNet() if is_crop else SegmentationNet()
    path=root/f'vision/models/{name}.onnx';onnx.checker.check_model(str(path))
    manifest=json.loads(path.with_suffix('.json').read_text())
    assert hashlib.sha256(path.read_bytes()).hexdigest()==manifest['sha256']
    if is_crop:
        if dataset=='proposal_crops':crop_x,crop_y=data['test_x'],data['test_y']
        else:
            crop_x,crop_y=crops(data['test_x'],data['test_y'],30)
            if manifest.get('normalize_contrast'):crop_x=np.stack([normalize_contrast(c) for c in crop_x])
        x=crop_x[:20]
    else:x=data['test_x'][:4]
    model.load_state_dict(torch.load(path.with_suffix('.pt'),weights_only=True));model.eval()
    options=ort.SessionOptions();options.intra_op_num_threads=4
    session=ort.InferenceSession(str(path),sess_options=options,providers=['CPUExecutionProvider'])
    maximum=0
    for sample in x:
        tensor=sample[None].astype(np.float32)/255
        with torch.no_grad():reference=model(torch.from_numpy(tensor)).numpy()
        actual=session.run(None,{session.get_inputs()[0].name:tensor})[0]
        maximum=max(maximum,float(np.abs(actual-reference).max()))
    assert maximum<1e-4,(name,maximum)
    errors[name]={'maximum_absolute_logit_difference':maximum,'samples':len(x),'onnx_sha256':manifest['sha256']}
    if is_crop:
        report_path=root/('reports/crop_training.json' if name=='crop_classifier' else 'reports/crop_night_training.json')
        report=json.loads(report_path.read_text())
        threshold=.9 if name=='crop_classifier' else .75
        report['test']=evaluate(model,DataLoader(TensorDataset(torch.from_numpy(crop_x),torch.from_numpy(crop_y)),batch_size=64),threshold)
        assert report['split_crops']['test']==len(crop_x)
        report_path.write_text(json.dumps(report,indent=2))
(root/'reports/onnx_verification.json').write_text(json.dumps(errors,indent=2))
print(json.dumps(errors,indent=2))
