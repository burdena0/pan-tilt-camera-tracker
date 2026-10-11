"""select the checkpoint that retained full silhouettes in the tracking evaluation."""
from pathlib import Path
import hashlib,json,shutil
import numpy as np,torch
from torch.utils.data import DataLoader,TensorDataset
from camera_tracker.model import SegmentationNet
from camera_tracker.train import scores
root=Path(__file__).resolve().parents[1];models=root/'vision/models'
torch.set_num_threads(4)
shutil.copyfile(models/'bootstrap_synthetic.pt',models/'synthetic_drone.pt')
model=SegmentationNet();model.load_state_dict(torch.load(models/'synthetic_drone.pt',weights_only=True));model.eval()
path=models/'synthetic_drone.onnx'
torch.onnx.export(model,torch.zeros(1,3,192,320),str(path),opset_version=13,dynamo=False,input_names=['rgb'],output_names=['logits'])
manifest=json.loads(path.with_suffix('.json').read_text());manifest['sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
path.with_suffix('.json').write_text(json.dumps(manifest,indent=2))
data=np.load(root/'vision/output/dataset_ray_checked.npz')
loader=DataLoader(TensorDataset(torch.from_numpy(data['test_x']),torch.from_numpy(data['test_y']).long()),batch_size=24)
report={'selection':'initial checkpoint retained; corrected-label adaptation improved pixel IoU but fragmented tracking proposals',
        'parameters':23427,'train_images':1600,'validation_images':200,'test_images':400,'initial_training_epochs':20,
        'checkpoint_sha256':hashlib.sha256(models.joinpath('synthetic_drone.pt').read_bytes()).hexdigest(),
        'test':scores(model,loader),'domain':'synthetic only; corrected ray-checked test masks'}
(root/'reports/selected_daylight_model.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
