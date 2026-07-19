import numpy as np
from indi_harness.sitl.frames import ned_to_enu, enu_to_ned

def test_known_vectors():
    assert np.allclose(ned_to_enu([1.0, 2.0, 3.0]), [2.0, 1.0, -3.0])
    assert np.allclose(enu_to_ned([2.0, 1.0, -3.0]), [1.0, 2.0, 3.0])

def test_involution_round_trip():
    v = np.array([0.3, -1.2, 4.5])
    assert np.allclose(enu_to_ned(ned_to_enu(v)), v)
    assert np.allclose(ned_to_enu(ned_to_enu(v)), v)  # swap is its own inverse
