"""split print-bed parts, keeping the original surface triangles."""
from pathlib import Path
import hashlib
import json
import trimesh
root = Path(__file__).resolve().parents[1]
source = root / 'simulation/assets/original'
output = root / 'simulation/assets/assembled'
output.mkdir(parents=True, exist_ok=True)
manifest = {'source': 'https://www.thingiverse.com/thing:4259522', 'creator': 'PJohnso1',
            'license': 'CC BY 4.0', 'units': 'mm', 'files': []}
for path in sorted(source.glob('*.stl')):
    mesh = trimesh.load_mesh(path)
    manifest['files'].append({'name': path.name, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                             'bounds_mm': mesh.bounds.tolist(), 'triangles': len(mesh.faces)})
    if path.name in ('Turret_bushings.stl', 'Turret_stepper_gears.stl'):
        parts = sorted(mesh.split(only_watertight=False), key=lambda m: m.bounds[:, 0].mean())
        for i, part in enumerate(parts):
            part.apply_translation([-part.bounds[:, 0].mean(), -part.bounds[:, 1].mean(), -part.bounds[0, 2]])
            part.export(output / f'{path.stem}_{i}.stl')
(root / 'simulation/assets/manifest.json').write_text(json.dumps(manifest, indent=2))
