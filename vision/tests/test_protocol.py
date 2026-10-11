import numpy as np
import pytest
from camera_tracker.protocol import packet,framed,parse_status
from camera_tracker.firmware_sim import FirmwareSim

def test_host_packets_run_through_actual_firmware_parser():
    firmware=FirmwareSim();firmware.enable(1)
    assert firmware.receive(packet(1234,1,1000,12,-5),1)
    assert firmware.tick(1.1)==[12,-5]
    assert firmware.tick(1.25)==[0,0]
    assert not firmware.guard.enabled
    assert not firmware.receive(packet(1234,2,1251,12,-5),1.251)

def test_corrupted_frame_latches_camera_motion_off():
    firmware=FirmwareSim();firmware.enable(1)
    assert firmware.receive(packet(1234,1,1000,12,5),1)
    bad=bytearray(packet(1234,2,1010,12,5));bad[12]^=1
    assert not firmware.receive(bytes(bad),1.01)
    assert firmware.command==[0,0] and not firmware.guard.enabled

def test_device_time_wraparound_and_old_session():
    firmware=FirmwareSim();firmware.enable(1,99)
    assert not firmware.receive(packet(1234,1,1000,0,0),1)
    firmware.enable(4294967.28,99)
    assert firmware.receive(packet(99,1,0xfffffff0,1,2),4294967.281)
    assert firmware.tick(4294967.4)==[1,2]

@pytest.mark.parametrize('pan,tilt',[(float('nan'),0),(0,float('inf')),(45.1,0),(0,30.1)])
def test_host_rejects_invalid_velocities(pan,tilt):
    with pytest.raises(ValueError):packet(1,1,1,pan,tilt)

def test_structured_telemetry_crc_and_bounds():
    status=parse_status(framed('@S1,1234,1000,4,12000,-5000,1,1,0,0'),2)
    assert status.pan==12 and status.tilt==-5 and status.enabled
    assert parse_status(framed('@S1,1234,1000,4,999999,-5000,1,1,0,0'),2) is None
    assert parse_status(b'@S1,1234,1000,4,12000,-5000,1,1,0,0*FFFF',2) is None
