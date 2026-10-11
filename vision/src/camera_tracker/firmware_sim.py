import ctypes
import os
from .simulation import ROOT

class Packet(ctypes.Structure):
    _fields_=[('session',ctypes.c_uint32),('sequence',ctypes.c_uint32),('capture_ms',ctypes.c_uint32),
              ('pan_mdegps',ctypes.c_int32),('tilt_mdegps',ctypes.c_int32),('ttl_ms',ctypes.c_uint32)]
class Guard(ctypes.Structure):
    _fields_=[('session',ctypes.c_uint32),('last_sequence',ctypes.c_uint32),('deadline_ms',ctypes.c_uint32),
              ('enabled',ctypes.c_bool),('received',ctypes.c_bool)]

class FirmwareSim:
    """runs the actual portable C parser and watchdog against Python host packets."""
    def __init__(self):
        name='vision_guard.dll' if os.name=='nt' else 'vision_guard.so'
        self.lib=ctypes.CDLL(str(ROOT/'vision/output'/name))
        self.lib.vision_parse.argtypes=[ctypes.c_char_p,ctypes.POINTER(Packet)];self.lib.vision_parse.restype=ctypes.c_bool
        self.lib.vision_guard_enable.argtypes=[ctypes.POINTER(Guard),ctypes.c_uint32,ctypes.c_uint32]
        self.lib.vision_guard_disable.argtypes=[ctypes.POINTER(Guard)]
        self.lib.vision_guard_accept.argtypes=[ctypes.POINTER(Guard),ctypes.POINTER(Packet),ctypes.c_uint32]
        self.lib.vision_guard_accept.restype=ctypes.c_bool
        self.lib.vision_guard_expired.argtypes=[ctypes.POINTER(Guard),ctypes.c_uint32];self.lib.vision_guard_expired.restype=ctypes.c_bool
        self.guard=Guard();self.command=[0.,0.];self.rejections=0
    def enable(self,now,session=1234):self.lib.vision_guard_enable(ctypes.byref(self.guard),session,round(now*1000))
    def disable(self):self.lib.vision_guard_disable(ctypes.byref(self.guard));self.command=[0.,0.]
    def receive(self,line,now):
        p=Packet()
        ok=self.lib.vision_parse(line.strip(),ctypes.byref(p)) and self.lib.vision_guard_accept(ctypes.byref(self.guard),ctypes.byref(p),round(now*1000))
        if ok:self.command=[p.pan_mdegps/1000,p.tilt_mdegps/1000]
        else:self.rejections+=1;self.disable()
        return ok
    def tick(self,now):
        if self.lib.vision_guard_expired(ctypes.byref(self.guard),round(now*1000)):self.disable()
        return self.command.copy()
