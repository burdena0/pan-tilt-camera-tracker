import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import threading
import time
import cv2
import numpy as np
from .detection import NeuralDetector
from .geometry import Calibration
from .protocol import Link
from .tracking import Tracker,CameraServo
from .image_quality import quality_metrics,exposure_budget_ms

@dataclass
class Frame:
    sequence:int
    timestamp:float
    image:np.ndarray

class LatestCapture:
    def __init__(self,index,size,raw_exposure=None,raw_auto_exposure=None,raw_gain=None):
        self.capture=cv2.VideoCapture(index,cv2.CAP_DSHOW)
        if not self.capture.isOpened():raise RuntimeError('USB camera could not be opened')
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH,size[0]);self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT,size[1])
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE,1)
        self.settings={}
        for name,prop,value in [('auto_exposure',cv2.CAP_PROP_AUTO_EXPOSURE,raw_auto_exposure),
                                ('exposure',cv2.CAP_PROP_EXPOSURE,raw_exposure),('gain',cv2.CAP_PROP_GAIN,raw_gain)]:
            if value is not None:
                if not np.isfinite(value) or not self.capture.set(prop,value):
                    self.capture.release();raise RuntimeError(f'camera rejected {name} setting')
                actual=self.capture.get(prop)
                if not np.isfinite(actual) or not np.isclose(actual,value,atol=.01):
                    self.capture.release();raise RuntimeError(f'camera {name} read-back differs: requested {value}, got {actual}')
                self.settings[name]={'requested':value,'driver_read_back':actual}
        self.frame=None;self.error=None;self.condition=threading.Condition();self.closed=False
        self.thread=threading.Thread(target=self._read,daemon=True);self.thread.start()
    def _read(self):
        sequence=0
        while not self.closed:
            ok,image=self.capture.read()
            stamp=time.monotonic()
            if not ok:
                self.error='camera read failed'
                with self.condition:self.condition.notify_all()
                return
            sequence+=1
            with self.condition:
                self.frame=Frame(sequence,stamp,image);self.condition.notify_all()
    def latest(self,previous):
        with self.condition:
            self.condition.wait_for(lambda:self.error or self.closed or (self.frame and self.frame.sequence!=previous),timeout=.15)
            if self.error:raise RuntimeError(self.error)
            if self.frame is None or self.frame.sequence==previous:raise RuntimeError('camera stalled')
            return self.frame
    def close(self):
        self.closed=True;self.capture.release()
        with self.condition:self.condition.notify_all()
        self.thread.join(timeout=1)

def overlay(image,tracks,calibration,pan,tilt,locked):
    output=image.copy()
    for t in tracks:
        color=(60,210,90) if t.id==locked else (220,170,50)
        if t.observed:
            x1,y1,x2,y2=t.bbox.astype(int)
            cv2.rectangle(output,(x1,y1),(x2,y2),color,1)
            cv2.putText(output,f'{t.label} #{t.id}',(x1,max(12,y1-4)),cv2.FONT_HERSHEY_SIMPLEX,.35,color,1)
        point=calibration.pixel(t.state[:2],pan,tilt)
        if point is not None:
            center=tuple(point.astype(int));cv2.circle(output,center,3,color,1)
    h,w=image.shape[:2];cv2.drawMarker(output,(w//2,h//2),(255,255,255),cv2.MARKER_CROSS,12,1)
    return output

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--model',type=Path,default=Path(__file__).resolve().parents[2]/'models/synthetic_drone.onnx')
    parser.add_argument('--calibration',type=Path,required=True)
    parser.add_argument('--camera',type=int,default=0)
    parser.add_argument('--port')
    parser.add_argument('--live',action='store_true')
    parser.add_argument('--raw-exposure',type=float)
    parser.add_argument('--raw-auto-exposure',type=float)
    parser.add_argument('--raw-gain',type=float)
    parser.add_argument('--log',type=Path,default=Path('camera_tracking.jsonl'))
    args=parser.parse_args()
    if args.raw_exposure is not None and args.raw_auto_exposure is None:
        parser.error('set the verified driver value for manual mode with --raw-auto-exposure when requesting --raw-exposure')
    calibration=Calibration.load(args.calibration)
    if args.live and (not args.port or calibration.source!='measured' or calibration.rms_px>.8):
        parser.error('live motion requires a serial port and measured camera calibration')
    detector=NeuralDetector(args.model);tracker=Tracker();servo=CameraServo()
    camera=None;link=None;log=None
    try:
        camera=LatestCapture(args.camera,calibration.size,args.raw_exposure,args.raw_auto_exposure,args.raw_gain)
        if args.live:
            link=Link(args.port);link.wait_ready()
        log=args.log.open('w');previous=0;previous_angles=None;previous_time=None
        log.write(json.dumps({'camera_driver_settings':camera.settings,'settings_units':'device/backend-specific; exposure timing unmeasured'})+'\n')
        while True:
            frame=camera.latest(previous);previous=frame.sequence
            if (frame.image.shape[1],frame.image.shape[0])!=calibration.size:raise RuntimeError('camera resolution differs from calibration')
            state=link.snapshot(frame.timestamp) if link else None
            pan,tilt=(state.pan,state.tilt) if state else (0.,0.)
            if previous_time is not None and frame.timestamp-previous_time>.12:detector.reset_temporal()
            detections,_=detector.detect(frame.image)
            tracks=tracker.update(detections,frame.timestamp,calibration,pan,tilt,mode=detector.active_mode)
            age=time.monotonic()-frame.timestamp
            command,reason=servo.command(tracks,frame.timestamp,age,pan,tilt)
            selected=next((t for t in tracks if t.id==servo.lock_id and t.observed),None)
            axis_speed=np.zeros(2) if previous_time is None else (np.array([pan,tilt])-previous_angles)/(frame.timestamp-previous_time)
            speed_px=float(np.linalg.norm(selected.state[2:]-axis_speed)*np.mean([calibration.K[0,0],calibration.K[1,1]])*np.pi/180) if selected else 0.
            budget=exposure_budget_ms(speed_px)
            quality=quality_metrics(frame.image)
            previous_time,previous_angles=frame.timestamp,np.array([pan,tilt])
            if link:
                if not state:raise RuntimeError('telemetry stalled or not synchronized with image')
                if state.enabled:link.send(command,frame.timestamp)
                else:reason='not homed; TRACK off, press h' if not state.homed else 'motion disabled; press e to enable'
            else:reason='preview only'
            image=overlay(frame.image,tracks,calibration,pan,tilt,servo.lock_id)
            cv2.putText(image,f'{reason} / frame {age*1000:.0f}ms / shutter budget {budget:.1f}ms',(8,image.shape[0]-10),cv2.FONT_HERSHEY_SIMPLEX,.4,(255,255,255),1)
            cv2.imshow('camera tracker / q quit, r reset lock, e enable, h home, c clear, space stop',image)
            log.write(json.dumps({'time':frame.timestamp,'sequence':frame.sequence,'age_ms':age*1000,'state':reason,'pan':pan,'tilt':tilt,'velocity':command.tolist(),'exposure_budget_ms':budget,'quality':quality,'tracks':[{'id':t.id,'label':t.label,'observed':t.observed,'sigma_deg':t.sigma_deg} for t in tracks]})+'\n')
            key=cv2.waitKey(1)&0xff
            if key==ord('q'):break
            if key==ord('r'):servo.lock_id=None
            if key==ord('e') and link:link.enable()
            if key==ord('h') and link:link.home()
            if key==ord('c') and link:link.clear()
            if key==ord(' ') and link:link.stop()
    finally:
        if link:link.close()
        if camera:camera.close()
        if log:log.close()
        cv2.destroyAllWindows()
if __name__=='__main__':main()
