import hashlib
import json
import mujoco
import numpy as np
from camera_tracker.simulation import ROOT,world_xml
from camera_tracker.geometry import camera_rotation

def test_original_mesh_hashes_and_two_axis_camera_attachment():
    manifest=json.loads((ROOT/'simulation/assets/manifest.json').read_text())
    assert len(manifest['files'])==5
    for item in manifest['files']:
        path=ROOT/'simulation/assets/original'/item['name']
        assert hashlib.sha256(path.read_bytes()).hexdigest()==item['sha256']
    model=mujoco.MjModel.from_xml_string(world_xml());data=mujoco.MjData(model)
    assert model.njnt==2 and model.nu==2
    assert model.cam_bodyid[model.camera('sensor').id]==model.body('tilt').id
    assert np.allclose(model.jnt_axis,[[0,0,1],[0,-1,0]])
    for pan,tilt in ((0,0),(20,15),(-40,45)):
        data.qpos[:]=np.deg2rad([pan,tilt]);mujoco.mj_forward(model,data)
        gl_rotation=data.cam_xmat[model.camera('sensor').id].reshape(3,3)
        assert np.allclose(gl_rotation@np.diag([1,-1,-1]),camera_rotation(pan,tilt),atol=1e-6)
        light=model.light('tracking_spotlight').id
        assert model.light_bodyid[light]==model.body('tilt').id
        assert np.allclose(data.light_xdir[light],camera_rotation(pan,tilt)[:,2],atol=1e-6)

def test_payload_interface_removed_from_firmware():
    assert not (ROOT/'main/payload.c').exists() and not (ROOT/'main/payload.h').exists()
    for path in (ROOT/'main').glob('*.[ch]'):
        code=path.read_text(encoding='utf-8-sig')
        for name in ('safety_request_fire','TRIGGER_PULL_US','PIN_TRIGGER_SERVO','payload_set_trigger'):
            assert name not in code
