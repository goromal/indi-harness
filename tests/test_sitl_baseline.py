import json
import numpy as np
from indi_harness.sitl.baseline import BATTERY, HoverStep, results_json

def test_battery_shape():
    names = [c.name for c in BATTERY]
    assert names == ["hover_step", "circle_slow", "circle_fast",
                     "lemniscate_slow", "lemniscate_fast"]
    assert all(c.duration > 0 for c in BATTERY)

def test_hover_step():
    h = HoverStep(point=np.array([0.0, 0.0, -10.0]), step_t=5.0,
                  step=np.array([2.0, 0.0, 0.0]))
    assert np.allclose(h.ref(4.9).p, [0, 0, -10])
    assert np.allclose(h.ref(5.1).p, [2, 0, -10])
    for t in (2.0, 8.0):
        fo = h.ref(t)
        for term in (fo.v, fo.a, fo.j, fo.s):
            assert np.allclose(term, 0)

def test_results_json_roundtrip():
    res = [{"case": "circle_slow", "rmse_total": 0.42,
            "rmse_axes": [0.1, 0.2, 0.3], "n_bin_samples": 1234,
            "source": "XKF1"}]
    s = results_json(res)
    assert json.loads(s)[0]["rmse_total"] == 0.42
