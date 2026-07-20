import numpy as np
from indi_harness.offboard.setpoints import AttitudeSetpoint, ATT_TYPEMASK_FULL


def test_typemask():
    assert ATT_TYPEMASK_FULL == 0x00


def test_fields():
    sp = AttitudeSetpoint(q=np.array([1.0, 0, 0, 0]),
                          w=np.array([0.1, 0.2, 0.3]), thrust=0.42)
    f = sp.fields(boot_ms=99)
    assert f["time_boot_ms"] == 99 and f["type_mask"] == 0
    assert f["q"] == [1.0, 0.0, 0.0, 0.0]
    assert (f["body_roll_rate"], f["body_pitch_rate"], f["body_yaw_rate"]) \
        == (0.1, 0.2, 0.3)
    assert f["thrust"] == 0.42


def test_thrust_clip():
    sp = AttitudeSetpoint(q=np.array([1.0, 0, 0, 0]), w=np.zeros(3), thrust=1.4)
    assert sp.fields()["thrust"] == 0.95
