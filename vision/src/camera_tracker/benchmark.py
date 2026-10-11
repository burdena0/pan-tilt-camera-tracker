import argparse
import json
from pathlib import Path
import cv2
import imageio.v2 as imageio
import numpy as np
from .simulation import Scene,ROOT
from .detection import NeuralDetector
from .tracking import Tracker,CameraServo
from .protocol import packet
from .firmware_sim import FirmwareSim
from .runtime import overlay

def bbox(mask):
    y,x=np.where(mask)
    return None if len(x)<5 else np.array([x.min(),y.min(),x.max()+1,y.max()+1],dtype=float)

def layout(scene,sensor,heatmap,caption,case,t,angles,command,locked):
    output=np.full((720,1280,3),(22,28,34),np.uint8)
    output[70:610,:800]=scene.overview(width=800,height=540)
    output[95:383,800:1280]=cv2.resize(sensor,(480,288),interpolation=cv2.INTER_NEAREST)
    colored=cv2.applyColorMap((np.clip(heatmap,0,1)*255).astype(np.uint8),cv2.COLORMAP_TURBO)
    output[400:640,860:1260]=cv2.resize(colored,(400,240))
    lines=[('camera-only tracking lab / original Thingiverse STL assembly',(26,33),.72),
           (f'{case} / t={t:.1f}s / {caption}',(26,670),.65),
           (f'pan {angles[0]:+6.2f} deg   tilt {angles[1]:+6.2f} deg',(26,699),.6),
           (f'camera view / locked ID {locked}',(812,81),.5),
           ('neural drone probability',(812,397),.5)]
    for text,where,size in lines:cv2.putText(output,text,where,cv2.FONT_HERSHEY_SIMPLEX,size,(226,235,242),1,cv2.LINE_AA)
    cv2.putText(output,f'velocity {command[0]:+.1f}, {command[1]:+.1f} deg/s',(800,699),cv2.FONT_HERSHEY_SIMPLEX,.5,(80,210,210),1)
    return output

def run_case(case,detector,video=None,duration=12,dt=.05):
    detector.reset_temporal()
    night=case.startswith('night_')
    scene=Scene(low_light=night); scene.set_angles(0,10)
    lamp=night and 'spotlight' in case
    scene.spotlight(lamp)
    exposure_ms=(2 if 'short_shutter' in case else 40) if night else None
    rng=np.random.default_rng(23456)
    tracker,servo,firmware=Tracker(),CameraServo(),FirmwareSim();firmware.enable(0)
    sequence=0;rows=[];first_watchdog=None;wrong_locks=0
    for frame in range(round(duration/dt)):
        t=frame*dt
        x=2.8 if case!='distant' else 5.5
        fast=case.startswith('night_fast_')
        frequency=1.8 if fast else .65
        elevation_frequency=2.4 if fast else .9
        y=.7*np.sin(frequency*t);z=.64+.2*np.sin(elevation_frequency*t)
        def pose_at(stamp):
            sy=.7*np.sin(frequency*stamp);sz=.64+.2*np.sin(elevation_frequency*stamp)
            scene.pose('drone0',[x,sy,sz] if not case.endswith('negative') else [0,0,-10],[.08*np.sin(stamp),.12,np.sin(stamp)*.5])
            scene.pose('drone1',[x+.4,-sy,sz+.25] if case=='distractors' else [0,0,-10],[.1,.12,-.4])
            scene.pose('bird',[3.2,.8*np.sin(.7*stamp+1),.8],[0,.1,.4])
        pose_at(t)
        occluded=case=='occlusion' and 4<t<4.5
        scene.pose('occluder',[1.4,y*.5,z*.5+.08] if occluded else [0,0,-10])
        image,_,instances=scene.render(annotation=True)
        if night:
            image=scene.render_exposure(pose_at,t,exposure_ms)
            # matched output gain, with more noise for the shorter exposure.
            noise=2*np.sqrt(40/exposure_ms)
            image=np.clip(image.astype(float)+rng.normal(0,noise,image.shape),0,255).astype(np.uint8)
        if case=='blur_dim':image=cv2.GaussianBlur((image.astype(float)*.5).astype(np.uint8),(5,5),1)
        angles=scene.angles
        stalled=case=='capture_stall' and 4<t<4.65
        if stalled:
            command=np.zeros(2);reason='capture stalled';probability=np.zeros((3,192,320))
            tracks=tracker.update([],t,scene.calibration,*angles,mode=detector.active_mode)
            detector.reset_temporal()
        else:
            detections,probability=detector.detect(image)
            tracks=tracker.update(detections,t,scene.calibration,*angles,mode=detector.active_mode)
            age=.2 if case=='stale_frame' and 4<t<4.3 else 0
            command,reason=servo.command(tracks,t,age,*angles)
            if firmware.guard.enabled:
                sequence+=1
                message=packet(firmware.guard.session,sequence,round((t-age)*1000),*command)
                if case=='corrupt_packet' and frame==81:message=message.replace(b'@V1',b'@V2')
                firmware.receive(message,t)
        command=np.array(firmware.tick(t))
        if case=='fixed_camera':command[:]=0;firmware.command=[0.,0.]
        if not firmware.guard.enabled:
            reason='watchdog / protocol latch; motion stopped'
            if first_watchdog is None:first_watchdog=t
        truth=bbox(instances==1)
        error=float(np.linalg.norm((truth[:2]+truth[2:])/2-[160,96])) if truth is not None else None
        selected=next((track for track in tracks if track.id==servo.lock_id),None)
        if selected and selected.observed and truth is not None:
            center=(truth[:2]+truth[2:])/2
            bird_box=bbox(instances==3)
            if bird_box is not None and np.linalg.norm(selected.bbox[:2]+selected.bbox[2:]-bird_box[:2]-bird_box[2:]) < np.linalg.norm(selected.bbox[:2]+selected.bbox[2:]-2*center):wrong_locks+=1
        rows.append({'time':t,'pan':float(angles[0]),'tilt':float(angles[1]),'pan_speed':float(np.rad2deg(scene.data.qvel[0])),
                     'tilt_speed':float(np.rad2deg(scene.data.qvel[1])),'error_px':error,'command':command.tolist(),
                     'enabled':bool(firmware.guard.enabled),'reason':reason,'lock_id':servo.lock_id,
                     'visible':truth is not None,'observed':bool(selected and selected.observed),
                     'illumination_expert':detector.active_mode,
                     'false_lock_without_visible_drone':bool(selected and selected.observed and not np.any((instances==1)|(instances==2)))})
        if video is not None:
            shown=overlay(image,tracks,scene.calibration,*angles,servo.lock_id)
            video.append_data(cv2.cvtColor(layout(scene,shown,probability[1],reason,case,t,angles,command,servo.lock_id),cv2.COLOR_BGR2RGB))
            if frame==120:cv2.imwrite(str(ROOT/f'reports/{case}_preview.png'),layout(scene,shown,probability[1],reason,case,t,angles,command,servo.lock_id))
        scene.step(command,dt,firmware=firmware,start_time=t)
    valid=[r['error_px'] for r in rows if r['error_px'] is not None and r['time']>1 and r['enabled']]
    settled=None
    if first_watchdog is not None:
        for i,r in enumerate(rows[:-4]):
            if r['time']>=first_watchdog and all(max(abs(v['pan_speed']),abs(v['tilt_speed']))<.1 for v in rows[i:i+5]):
                settled=r['time'];break
    result={'case':case,'frames':len(rows),'mean_center_error_px':float(np.mean(valid)) if valid else None,
            'p95_center_error_px':float(np.percentile(valid,95)) if valid else None,
            'center_error_evaluation_frames':len(valid),'pan_range_deg':[min(r['pan'] for r in rows),max(r['pan'] for r in rows)],
            'tilt_range_deg':[min(r['tilt'] for r in rows),max(r['tilt'] for r in rows)],
            'visible_frames':sum(r['visible'] for r in rows),'observed_locked_frames':sum(r['observed'] for r in rows),
            'possible_bird_handover_frames':wrong_locks,'first_disable_seconds':first_watchdog,
            'false_lock_without_visible_drone_frames':sum(r['false_lock_without_visible_drone'] for r in rows),
            'peak_axis_speed_dps':[max(abs(r['pan_speed']) for r in rows),max(abs(r['tilt_speed']) for r in rows)],
            'spotlight':lamp,'exposure_ms':exposure_ms,'settled_stop_seconds':settled,
            'motion_frequencies_hz':[frequency/(2*np.pi),elevation_frequency/(2*np.pi)],
            'peak_lateral_speed_mps':.7*frequency,
            'protocol_rejections':firmware.rejections,
            'nonzero_commands_when_disabled':sum(bool(any(r['command'])) for r in rows if not r['enabled'])}
    scene.close();return result,rows

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--video',action='store_true');parser.add_argument('--cases',nargs='+')
    args=parser.parse_args();reports=ROOT/'reports';reports.mkdir(exist_ok=True)
    detector=NeuralDetector(ROOT/'vision/models/synthetic_drone.onnx')
    cases=args.cases or ['fixed_camera','nominal','distractors','occlusion','capture_stall','corrupt_packet','stale_frame','negative','blur_dim','distant',
                        'night_long_shutter','night_spotlight','night_short_shutter','night_spotlight_short_shutter',
                        'night_spotlight_short_shutter_negative','night_fast_spotlight_short_shutter']
    results=[];traces={}
    for case in cases:
        writer=imageio.get_writer(str(reports/f'{case}.mp4'),fps=20,codec='libx264',quality=8,macro_block_size=1) if args.video and case in ('nominal','occlusion','capture_stall','distractors','night_long_shutter','night_spotlight_short_shutter') else None
        try:result,rows=run_case(case,detector,writer)
        finally:
            if writer is not None:writer.close()
        results.append(result);traces[case]=rows;print(json.dumps(result),flush=True)
    timing=np.array(detector.timings)
    report={'domain':'synthetic CAD-based camera-only simulation','camera_resolution':[320,192],
            'controller_uses_ground_truth':False,'model_sha256':detector.manifest['sha256'],
            'verifier_sha256':detector.manifest.get('verifier',{}).get('sha256'),
            'illumination_expert':detector.manifest.get('illumination_expert'),
            'lighting_model':'Phong spotlight, estimated attenuation; not calibrated lux, LED wattage or sensor photon counts',
            'shutter_model':'5 temporal samples; constant instantaneous camera velocity during exposure; matched gain and normalized Gaussian noise',
            'cpu_inference_ms':{'median':float(np.median(timing)), 'p95':float(np.percentile(timing,95))},
            'results':results}
    (reports/'vision_simulation.json').write_text(json.dumps(report,indent=2))
    (reports/'vision_traces.json').write_text(json.dumps(traces))
if __name__=='__main__':main()
