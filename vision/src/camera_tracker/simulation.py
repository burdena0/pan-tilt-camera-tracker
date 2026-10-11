from pathlib import Path
import cv2
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from .geometry import Calibration

ROOT = Path(__file__).resolve().parents[3]

def drone_body(name, position):
    parts = [f'<body name="{name}" mocap="true" pos="{position}">',
             '<geom type="box" size=".045 .03 .016" rgba=".08 .09 .1 1"/>']
    for x in (-.105, .105):
        for y in (-.105, .105):
            parts.append(f'<geom type="capsule" fromto="0 0 0 {x} {y} 0" size=".009" rgba=".2 .2 .2 1"/>')
            parts.append(f'<geom type="cylinder" pos="{x} {y} .008" size=".042 .003" rgba=".08 .08 .09 1"/>')
    return ''.join(parts) + '</body>'

def world_xml(low_light=False):
    assets = (ROOT / 'simulation/assets').as_posix()
    bushes = ''.join(f'<geom type="mesh" mesh="bush" pos="{.062*np.cos(a)} {.062*np.sin(a)} .013" rgba=".2 .2 .25 1"/>' for a in np.linspace(0, 2*np.pi, 6, endpoint=False))
    sky1,sky2=('.045 .065 .085','.088 .093 .098') if low_light else ('.45 .65 .85','.88 .93 .98')
    daylight='.06 .06 .06' if low_light else '.85 .85 .85'
    headlight='<headlight active="0"/>' if low_light else ''
    return f'''<mujoco model="camera tracker"><compiler angle="degree"/>
    <option timestep=".002" gravity="0 0 -9.81" integrator="implicitfast"/>
    <visual><global offwidth="1280" offheight="720"/><quality shadowsize="2048"/><map znear=".005" zfar="100"/>{headlight}</visual>
    <default><geom contype="0" conaffinity="0"/><joint damping=".02"/></default>
    <asset>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="{sky1}" rgb2="{sky2}" width="512" height="512"/>
    <texture name="groundtex" type="2d" builtin="checker" rgb1=".3 .36 .28" rgb2=".32 .38 .3" width="256" height="256"/>
    <material name="ground" texture="groundtex" texrepeat="20 20" reflectance="0"/>
    <mesh name="base" file="{assets}/original/Turret_base.stl" scale=".001 .001 .001"/>
    <mesh name="top" file="{assets}/original/Turret_top.stl" scale=".001 .001 .001"/>
    <mesh name="shaft" file="{assets}/original/Turret_Shaft.stl" scale=".001 .001 .001"/>
    <mesh name="bush" file="{assets}/assembled/Turret_bushings_0.stl" scale=".001 .001 .001"/>
    <mesh name="gear0" file="{assets}/assembled/Turret_stepper_gears_0.stl" scale=".001 .001 .001"/>
    <mesh name="gear1" file="{assets}/assembled/Turret_stepper_gears_1.stl" scale=".001 .001 .001"/>
    </asset><worldbody>
    <light name="environment" pos="2 -2 6" dir="-.3 .2 -1" diffuse="{daylight}" ambient="0 0 0"/>
    <geom type="plane" size="20 20 .1" pos="0 0 -.012" material="ground"/>
    <geom type="mesh" mesh="base" rgba=".15 .27 .34 1"/>
    {bushes}
    <body name="pan"><joint name="pan" axis="0 0 1" range="-150 150"/>
      <inertial pos="0 0 .07" mass=".4" diaginertia=".004 .004 .003"/>
      <geom type="mesh" mesh="top" rgba=".32 .58 .67 1"/>
      <geom type="mesh" mesh="gear0" pos="-.095 0 .008" rgba=".2 .2 .25 1"/>
      <geom type="box" pos="-.095 0 .038" size=".021 .021 .018" rgba=".12 .14 .17 1"/>
      <geom type="mesh" mesh="gear1" pos="0 -.062 .0802" quat=".7071068 -.7071068 0 0" rgba=".2 .2 .25 1"/>
      <geom type="box" pos="0 -.084 .0802" size=".021 .018 .021" rgba=".12 .14 .17 1"/>
      <body name="tilt" pos="0 0 .1302"><joint name="tilt" axis="0 -1 0" range="-25 60"/>
        <inertial pos=".015 0 .018" mass=".25" diaginertia=".0005 .0006 .0005"/>
        <geom type="mesh" mesh="shaft" pos="-.1 -.055 .1" quat=".7071068 -.7071068 0 0" rgba=".78 .58 .26 1"/>
        <geom name="camera_mount" type="box" pos=".015 0 .011" size=".027 .018 .004" rgba=".18 .19 .21 1"/>
        <geom name="camera_case" type="box" pos=".025 0 .027" size=".015 .021 .012" rgba=".07 .08 .1 1"/>
        <geom name="camera_lens" type="cylinder" pos=".043 0 .027" size=".008 .004" quat=".7071068 0 .7071068 0" rgba=".05 .2 .4 1"/>
        <camera name="sensor" pos=".048 0 .027" xyaxes="0 -1 0 0 0 1" fovy="42"/>
        <geom name="spotlight_case" type="cylinder" pos=".029 0 .058" size=".012 .014" quat=".7071068 0 .7071068 0" rgba=".18 .18 .2 1"/>
        <geom name="spotlight_lens" type="cylinder" pos=".044 0 .058" size=".01 .001" quat=".7071068 0 .7071068 0" rgba=".8 .75 .35 1"/>
        <light name="tracking_spotlight" type="spot" pos=".047 0 .058" dir="1 0 0" cutoff="28" exponent="8"
               attenuation="1 .03 .12" diffuse="4 3.8 3.5" specular="0 0 0" ambient="0 0 0" active="false" castshadow="false"/>
      </body>
    </body>
    {drone_body('drone0', '4 0 1')}{drone_body('drone1', '5 -1 1.4')}
    <body name="bird" mocap="true" pos="4 1 1">
      <geom type="ellipsoid" size=".055 .02 .022" rgba=".12 .1 .09 1"/>
      <geom type="capsule" fromto="0 0 0 -.045 .13 .028" size=".016" rgba=".14 .12 .1 1"/>
      <geom type="capsule" fromto="0 0 0 -.045 -.13 .028" size=".016" rgba=".14 .12 .1 1"/>
    </body>
    <body name="occluder" mocap="true" pos="0 0 -10"><geom type="box" size=".02 .8 .8" rgba=".45 .5 .5 1"/></body>
    </worldbody><actuator>
    <position name="pan_motor" joint="pan" kp="20" kv="1" ctrlrange="-2.617994 2.617994" forcerange="-3 3"/>
    <position name="tilt_motor" joint="tilt" kp="20" kv="1" ctrlrange="-.436332 1.047198" forcerange="-1.65 1.65"/>
    </actuator></mujoco>'''

class Scene:
    def __init__(self, width=320, height=192, low_light=False):
        self.model = mujoco.MjModel.from_xml_string(world_xml(low_light))
        self.data = mujoco.MjData(self.model)
        self.renderer = mujoco.Renderer(self.model, height=height, width=width)
        self.width, self.height = width, height
        focal = height / (2*np.tan(np.deg2rad(21)))
        self.calibration = Calibration([[focal, 0, width/2], [0, focal, height/2], [0, 0, 1]], np.zeros(5), (width, height))
        self.labels = np.zeros(self.model.ngeom, dtype=np.uint8)
        self.instance = np.zeros(self.model.ngeom, dtype=np.uint8)
        for body, label, instance in [('drone0',1,1),('drone1',1,2),('bird',2,3)]:
            bid = self.model.body(body).id
            selection = self.model.geom_bodyid == bid
            self.labels[selection], self.instance[selection] = label, instance
        self.planned_velocity = np.zeros(2)
        self.planned_position = np.zeros(2)
        self.overview_renderer = None
        mujoco.mj_forward(self.model, self.data)

    def spotlight(self,enabled):
        self.model.light_active[self.model.light('tracking_spotlight').id]=bool(enabled)

    def pose(self, name, xyz, rpy=None):
        body = self.model.body(name).id
        mocap = self.model.body_mocapid[body]
        self.data.mocap_pos[mocap] = xyz
        if rpy is not None:
            q = Rotation.from_euler('xyz', rpy).as_quat()
            self.data.mocap_quat[mocap] = q[[3,0,1,2]]

    def set_angles(self, pan, tilt):
        self.data.qpos[:2] = np.deg2rad([pan, tilt]); self.data.qvel[:2] = 0
        self.planned_velocity[:] = 0
        self.planned_position[:] = [pan,tilt]
        self.data.ctrl[:] = self.data.qpos[:2]
        mujoco.mj_forward(self.model, self.data)

    @property
    def angles(self): return np.rad2deg(self.data.qpos[:2]).copy()

    def render(self, annotation=False):
        mujoco.mj_forward(self.model, self.data)
        self.renderer.update_scene(self.data, camera='sensor')
        rgb = self.renderer.render().copy()
        if not annotation: return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        self.renderer.enable_segmentation_rendering()
        self.renderer.update_scene(self.data, camera='sensor')
        segmentation = self.renderer.render().copy()
        self.renderer.disable_segmentation_rendering()
        ids = segmentation[:,:,0]
        valid = (ids >= 0) & (segmentation[:,:,1] == int(mujoco.mjtObj.mjOBJ_GEOM))
        labels, instances = np.zeros(ids.shape,np.uint8),np.zeros(ids.shape,np.uint8)
        labels[valid],instances[valid] = self.labels[ids[valid]],self.instance[ids[valid]]
        # reject renderer ID artifacts where a ray actually hits ground or an occluder.
        ys,xs = np.where(labels != 0)
        if len(xs):
            cam = self.model.camera('sensor').id
            focal = self.calibration.K[0,0]
            rays = np.stack([(xs+.5-self.width/2)/focal,
                             -(ys+.5-self.height/2)/focal,-np.ones(len(xs))],axis=1)
            rays = rays @ self.data.cam_xmat[cam].reshape(3,3).T
            rays /= np.linalg.norm(rays,axis=1)[:,None]
            hits,distances = np.full(len(xs),-1,np.int32),np.zeros(len(xs))
            mujoco.mj_multiRay(self.model,self.data,self.data.cam_xpos[cam],rays.reshape(-1),
                              None,True,self.model.body('tilt').id,hits,distances,None,len(xs),100)
            hit_labels,hit_instances = np.zeros(len(xs),np.uint8),np.zeros(len(xs),np.uint8)
            found = hits >= 0
            hit_labels[found],hit_instances[found] = self.labels[hits[found]],self.instance[hits[found]]
            labels[ys,xs],instances[ys,xs] = hit_labels,hit_instances
        return cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR),labels,instances

    def render_exposure(self,pose_at,t,exposure_ms,samples=5):
        if exposure_ms<=0 or samples<2:raise ValueError('positive exposure and multiple shutter samples required')
        qpos,mocap_pos,mocap_quat=self.data.qpos.copy(),self.data.mocap_pos.copy(),self.data.mocap_quat.copy()
        images=[]
        try:
            for offset in np.linspace(-exposure_ms/1000,0,samples):
                self.data.qpos[:]=qpos+self.data.qvel*offset
                pose_at(t+offset)
                images.append(self.render())
        finally:
            self.data.qpos[:]=qpos;self.data.mocap_pos[:]=mocap_pos;self.data.mocap_quat[:]=mocap_quat
            mujoco.mj_forward(self.model,self.data)
        return np.mean(images,axis=0).astype(np.uint8)

    def step(self, command, dt, firmware=None, start_time=0):
        for _ in range(round(dt/self.model.opt.timestep)):
            if firmware is not None:
                command = np.asarray(firmware.tick(start_time + _*self.model.opt.timestep))
            self.planned_velocity += np.clip(command-self.planned_velocity,
                -np.array([80,55])*self.model.opt.timestep, np.array([80,55])*self.model.opt.timestep)
            self.planned_position += self.planned_velocity*self.model.opt.timestep
            self.planned_position = np.clip(self.planned_position,[-150,-25],[150,60])
            self.data.ctrl[:] = np.deg2rad(self.planned_position)
            mujoco.mj_step(self.model,self.data)

    def overview(self, width=960, height=540):
        if self.overview_renderer is None:
            self.overview_renderer=mujoco.Renderer(self.model,height=height,width=width)
        cam=mujoco.MjvCamera(); cam.lookat[:]=[0,0,.095]
        cam.distance,cam.azimuth,cam.elevation=.68,135,-22
        self.overview_renderer.update_scene(self.data,camera=cam)
        return cv2.cvtColor(self.overview_renderer.render(),cv2.COLOR_RGB2BGR)

    def close(self):
        self.renderer.close()
        if self.overview_renderer is not None:self.overview_renderer.close()
