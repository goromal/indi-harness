import json
import struct

import numpy as np

from indi_harness.params import QuadParams
from indi_harness.sitl.jsonsim.model import QuadJsonModel
from indi_harness.sitl.jsonsim.protocol import (
    LockstepDriver,
    format_state,
    parse_servo,
)

P = QuadParams()
DT = 1.0 / 400.0


def _pack(frame_rate, frame_count, pwm=None):
    if pwm is None:
        pwm = [1500] * 16
    return struct.pack("<HHI16H", 18458, frame_rate, frame_count, *pwm)


def test_parse_servo_ok():
    data = _pack(400, 7, [1000 + i for i in range(16)])
    pkt = parse_servo(data)
    assert pkt is not None
    assert pkt.frame_rate == 400
    assert pkt.frame_count == 7
    assert pkt.pwm == [1000 + i for i in range(16)]


def test_parse_servo_wrong_magic():
    bad = struct.pack("<HHI16H", 1, 400, 7, *([1500] * 16))
    assert parse_servo(bad) is None


def test_parse_servo_too_short():
    assert parse_servo(b"\x00\x01") is None


def test_format_state_round_trip():
    st = {
        "timestamp": 1.234567891,
        "position": [1.0, 2.0, 3.0],
        "velocity": [0.1, 0.2, 0.3],
        "quaternion": [1.0, 0.0, 0.0, 0.0],
        "gyro": [0.01, 0.02, 0.03],
        "accel_body": [0.0, 0.0, -9.81],
        "erpm": [100.0, 200.0, 300.0, 400.0],
    }
    s = format_state(st)
    decoded = json.loads(s)
    assert abs(decoded["timestamp"] - st["timestamp"]) < 1e-9
    assert decoded["imu"]["gyro"] == st["gyro"]
    assert decoded["imu"]["accel_body"] == st["accel_body"]
    assert decoded["position"] == st["position"]
    assert decoded["velocity"] == st["velocity"]
    assert decoded["quaternion"] == st["quaternion"]
    assert decoded["rpm_1"] == st["erpm"][0]
    assert decoded["rpm_2"] == st["erpm"][1]
    assert decoded["rpm_3"] == st["erpm"][2]
    assert decoded["rpm_4"] == st["erpm"][3]
    assert "imu" not in decoded or set(decoded.keys()) == {
        "timestamp",
        "imu",
        "position",
        "velocity",
        "quaternion",
        "rpm_1",
        "rpm_2",
        "rpm_3",
        "rpm_4",
    }


def test_reply_carries_erpm():
    m = QuadJsonModel(P, drag_on=False, dt=DT)
    wh = P.hover_speed()
    m.seed_omega(np.full(4, wh))
    m.step_omega(np.full(4, wh))
    reply = format_state(m.state())
    assert reply.endswith("\n")
    doc = json.loads(reply)
    for k in ("rpm_1", "rpm_2", "rpm_3", "rpm_4"):
        assert k in doc and doc[k] > 0
    assert abs(doc["rpm_1"] - P.omega_to_erpm(wh)) < 1.0


def test_lockstep_driver_steps_and_resets():
    m = QuadJsonModel(P, drag_on=False, dt=DT)
    wh = P.hover_speed()
    m.seed_omega(np.full(4, wh))
    driver = LockstepDriver(m)

    pwm_hover = [1000] * 16
    # pwm_to_omega maps u=(pwm-1000)/1000; to get hover speed we need
    # u = (wh/Omega_max)**2 -> pwm = 1000 + 1000*u
    u_hover = (wh / P.Omega_max) ** 2
    pwm_val = int(1000 + 1000 * u_hover)
    pwm_hover = [pwm_val] * 16

    reply1 = driver.on_packet(_pack(400, 1, pwm_hover))
    assert reply1 is not None
    st1 = json.loads(reply1)
    assert st1["timestamp"] > 0.0
    t_after_1 = st1["timestamp"]

    reply2 = driver.on_packet(_pack(400, 2, pwm_hover))
    st2 = json.loads(reply2)
    assert st2["timestamp"] > t_after_1

    # frame_count decreases -> reset
    reply3 = driver.on_packet(_pack(400, 1, pwm_hover))
    st3 = json.loads(reply3)
    assert st3["timestamp"] <= t_after_1
