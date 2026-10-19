import numpy as np

from indi_harness.sitl.guided_handover_probe import requested_parameters


def test_actuator_map_is_the_only_parameter_difference():
    normal = requested_parameters("normal")
    linearized = requested_parameters("linearized")

    assert set(linearized) - set(normal) == {
        "MOT_THST_EXPO", "MOT_SPIN_ARM", "MOT_SPIN_MIN", "MOT_SPIN_MAX",
        "MOT_BAT_VOLT_MIN", "MOT_BAT_VOLT_MAX",
    }
    for name in normal:
        assert np.isclose(normal[name], linearized[name])


def test_explicit_parameter_override_is_recorded():
    params = requested_parameters("linearized", ["CC3_G1_YAW=29.5"])
    assert params["CC3_G1_YAW"] == 29.5
