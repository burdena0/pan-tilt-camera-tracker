"""record the local caches and trained artifacts without publishing large datasets."""
from pathlib import Path
import hashlib,json
import numpy as np
root=Path(__file__).resolve().parents[1]
cache_rows=[]
for name in ('dataset','dataset_ray_checked','dataset_lighting','proposal_crops'):
    path=root/f'vision/output/{name}.npz'
    if not path.exists():continue
    data=np.load(path)
    cache_rows.append({'file':name+'.npz','sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
                       'split_counts':{split:int(len(data[split+'_x'])) for split in ('train','val','test')},
                       'annotation':'legacy renderer candidates' if name=='dataset' else 'ray-checked labels or their derived proposal crops'})
(root/'reports/data_lineage.json').write_text(json.dumps({'caches':cache_rows,
    'bootstrap':'the initial 20-epoch daylight checkpoint uses the legacy cached dataset; current generators use corrected ray checks',
    'benchmark':'known development trajectories, not a blind test; early night regression used the faster trajectory now named night_fast'},indent=2))
path=root/'reports/proposal_dataset.json';proposal=json.loads(path.read_text())
proposal.update(segmentation_sha256=json.loads((root/'vision/models/night_segmentation.json').read_text())['sha256'],contrast_close_kernel=5)
path.write_text(json.dumps(proposal,indent=2))
path=root/'reports/crop_night_training.json';report=json.loads(path.read_text())
report['evaluation']='actual detector proposals plus jittered geometric crops; source scene splits remain disjoint'
path.write_text(json.dumps(report,indent=2))
artifacts={}
for path in sorted((root/'vision/models').glob('*')):
    if path.is_file():
        contents=path.read_bytes()
        if path.suffix=='.json':contents=contents.replace(b'\r\n',b'\n')
        artifacts[path.name]={'sha256':hashlib.sha256(contents).hexdigest(),'bytes':len(contents)}
(root/'reports/model_artifacts.json').write_text(json.dumps(artifacts,indent=2))
print('recorded cache lineage and model artifact hashes')
