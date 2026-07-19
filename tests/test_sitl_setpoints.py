import numpy as np
from indi_harness.sitl.setpoints import (Setpoint, PVA_TYPEMASK,
                                         MAV_FRAME_LOCAL_NED)

def test_typemask_value():
    assert PVA_TYPEMASK == 0xC00

def test_field_packing():
    sp = Setpoint(p=np.array([1.0, 2.0, -5.0]), v=np.array([0.1, 0.2, 0.3]),
                  a=np.array([0.01, 0.02, 0.03]))
    f = sp.fields(boot_ms=1234)
    assert f["time_boot_ms"] == 1234
    assert f["coordinate_frame"] == MAV_FRAME_LOCAL_NED == 1
    assert f["type_mask"] == PVA_TYPEMASK
    assert (f["x"], f["y"], f["z"]) == (1.0, 2.0, -5.0)
    assert (f["vx"], f["vy"], f["vz"]) == (0.1, 0.2, 0.3)
    assert (f["afx"], f["afy"], f["afz"]) == (0.01, 0.02, 0.03)
    assert f["yaw"] == 0.0 and f["yaw_rate"] == 0.0
