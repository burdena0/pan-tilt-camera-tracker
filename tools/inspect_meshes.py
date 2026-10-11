from pathlib import Path
import cv2
import mujoco

root = Path(__file__).resolve().parents[1]
assets = (root / 'simulation/assets/original').as_posix()
xml = f'''<mujoco><visual><global offwidth="800" offheight="600"/></visual><asset>
<mesh name="base" file="{assets}/Turret_base.stl" scale=".001 .001 .001"/>
<mesh name="top" file="{assets}/Turret_top.stl" scale=".001 .001 .001"/>
</asset><worldbody><light pos="0 0 1"/>
<geom type="mesh" mesh="base" rgba=".2 .4 .5 1" contype="0" conaffinity="0"/>
<geom type="mesh" mesh="top" rgba=".5 .7 .8 1" contype="0" conaffinity="0"/>
</worldbody></mujoco>'''
model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
with mujoco.Renderer(model, height=600, width=800) as renderer:
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0, 0, .08]
    camera.distance, camera.azimuth, camera.elevation = .55, 45, -20
    renderer.update_scene(data, camera=camera)
    cv2.imwrite(str(root / 'simulation/mesh_inspection.png'), cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR))
