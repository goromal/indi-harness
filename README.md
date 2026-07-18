# indi-harness

Quaternion INDI prototype and trajectory harness for ArduPilot. This package implements the S0/S1 deliverable of the ArduPilot INDI plan: an offline control-law prototype using quaternion-based attitude control with a trajectory harness for testing and validation.

## Conventions

- **World frame:** North-East-Down (NED)
- **Body frame:** Forward-Right-Down (FRD)
- **Quaternions:** Hamilton scalar-first `[w, x, y, z]`, representing rotations from body frame to world frame
- **Dependencies:** Pure NumPy (no ROS/ArduPilot required for this stage)

## Running Tests

```bash
nix-shell --run "pytest tests/ -v"
```

All tests use Hamilton quaternion conventions and NED/FRD coordinate frames established in `indi_harness.quat`.
