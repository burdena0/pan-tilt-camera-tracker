from dataclasses import dataclass
from collections import deque
import binascii
import math
import threading
import time
import serial

def framed(body):
    data=body.encode('ascii')
    return data+b'*'+f'{binascii.crc_hqx(data,0xffff):04X}'.encode('ascii')+b'\n'

def packet(session,sequence,capture_ms,pan,tilt,ttl_ms=250):
    if not all(math.isfinite(v) for v in (pan,tilt)) or abs(pan)>45 or abs(tilt)>30:
        raise ValueError('invalid camera velocity')
    return framed(f'@V1,{session},{sequence},{capture_ms & 0xffffffff},{round(pan*1000)},{round(tilt*1000)},{ttl_ms}')

@dataclass
class Telemetry:
    session:int
    device_ms:int
    sequence:int
    pan:float
    tilt:float
    homed:bool
    enabled:bool
    faults:int
    rejected:int
    received:float

def parse_status(line,received):
    start=line.find(b'@S1,')
    if start<0:return None
    line=line[start:].strip()
    try:
        body,crc=line.split(b'*')
        if len(crc)!=4 or binascii.crc_hqx(body,0xffff)!=int(crc,16):return None
        parts=body.decode('ascii').split(',')
        if len(parts)!=10:return None
        v=list(map(int,parts[1:]))
        if any(n<0 or n>0xffffffff for n in (v[0],v[1],v[2],v[7],v[8])):return None
        if v[5] not in (0,1) or v[6] not in (0,1) or not -150000<=v[3]<=150000 or not -25000<=v[4]<=60000:return None
        return Telemetry(v[0],v[1],v[2],v[3]/1000,v[4]/1000,bool(v[5]),bool(v[6]),v[7],v[8],received)
    except (ValueError,UnicodeError):return None

class Link:
    def __init__(self,port):
        self.serial=serial.Serial(baudrate=115200,timeout=.05,write_timeout=.05)
        self.serial.dtr=False;self.serial.rts=False;self.serial.port=port;self.serial.open()
        self.history=deque(maxlen=100);self.sequence=0;self.error=None
        self.lock=threading.Lock();self.closed=threading.Event()
        self.thread=threading.Thread(target=self._read,daemon=True);self.thread.start()
    def _read(self):
        try:
            while not self.closed.is_set():
                line=self.serial.read_until(b'\n',size=256)
                status=parse_status(line,time.monotonic())
                if status:
                    with self.lock:self.history.append(status)
        except (serial.SerialException,OSError) as exc:self.error=str(exc)
    def snapshot(self,at_time=None):
        with self.lock:history=list(self.history)
        if not history or self.error or time.monotonic()-history[-1].received>.15:return None
        latest=history[-1]
        if at_time is None:return latest
        nearest=min(history,key=lambda t:abs(t.received-at_time))
        return nearest if abs(nearest.received-at_time)<=.06 else None
    def enable(self):self.serial.write(b'track on\n')
    def home(self):self.serial.write(b'home\n')
    def clear(self):self.serial.write(b'clear\n')
    def stop(self):self.serial.write(b'track off\n')
    def wait_ready(self,timeout=5):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if self.snapshot() is not None:return
            if self.error:raise RuntimeError(self.error)
            time.sleep(.02)
        raise RuntimeError('no valid firmware telemetry after opening serial port')
    def send(self,command,frame_time):
        state=self.snapshot()
        if state is None or not state.enabled or not state.homed or state.faults:
            raise RuntimeError('fresh, enabled, homed telemetry is required')
        age=time.monotonic()-frame_time
        if not 0<=age<=.12:command=[0,0];frame_time=time.monotonic()
        capture_ms=state.device_ms+round((frame_time-state.received)*1000)
        self.sequence=(self.sequence+1)&0xffffffff
        self.serial.write(packet(state.session,self.sequence,capture_ms,*command))
    def close(self):
        try:self.serial.write(b'track off\n')
        except (serial.SerialException,OSError):pass
        finally:
            self.closed.set();self.thread.join(timeout=.2);self.serial.close()
