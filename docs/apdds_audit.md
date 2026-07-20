# AP_DDS topic audit (S2 prerequisite)

Empirical inventory of the `/ap/*` ROS 2 graph that ArduPilot's AP_DDS client
publishes in the drone SITL VM. The design doc (§S2) requires this audit before
any offboard node code assumes a topic exists. Captured with the interactive
`dronesim.nix` test driver (`ros-pkgs.rosPackages.jazzy` `ros-core` guest env,
ArduCopter SITL, Micro-XRCE-DDS agent), vehicle idle on the ground (pre-arm).

## Method

- `ros2 topic list -t` for names + types.
- `ros2 topic hz -w 20 <topic>` over a ~12 s window for measured rates.
  **Limitation:** `ros2 topic hz` can only introspect topics whose message
  package is present in the guest. The minimal `ros-core` env lacks
  `ardupilot_msgs` and `geographic_msgs`, so hz on the custom-typed topics
  (`/ap/airspeed`, `/ap/cmd_gps_pose`, `/ap/geopose/filtered`, `/ap/goal_lla`,
  `/ap/gps_global_origin/filtered`, `/ap/rc`, `/ap/status`) aborts with
  "message type invalid". Rates below are for the standard-typed topics that S2
  actually consumes; the custom-typed ones are inventoried by type/QoS only.
- `ros2 topic info -v` for QoS and publisher/subscriber counts.
- `pymavlink` on the router GCS port (5790) for `LOCAL_POSITION_NED.time_boot_ms`
  to compare stamp epochs.

## Topic inventory

| Topic | Type | Measured rate (topic hz) | Role for S2 |
|-------|------|--------------------------|-------------|
| `/ap/pose/filtered` | `geometry_msgs/msg/PoseStamped` | **27.6 Hz** | **primary** — node ticks on this (ENU/FLU pose) |
| `/ap/twist/filtered` | `geometry_msgs/msg/TwistStamped` | **26.9 Hz** | velocity feedback (ENU) |
| `/ap/imu/experimental/data` | `sensor_msgs/msg/Imu` | **126.1 Hz** | **INDI specific-force** (FLU accel) |
| `/ap/time` | `builtin_interfaces/msg/Time` | **70.6 Hz** | sim-time bridge; recorded into every bag (§L) |
| `/ap/clock` | `rosgraph_msgs/msg/Clock` | **73.8 Hz** | ROS `/clock` source |
| `/ap/battery` | `sensor_msgs/msg/BatteryState` | 1.0 Hz | unused |
| `/ap/navsat` | `sensor_msgs/msg/NavSatFix` | 4.9 Hz | unused |
| `/ap/geopose/filtered` | `geographic_msgs/msg/GeoPoseStamped` | (custom type; not introspectable) | unused |
| `/ap/gps_global_origin/filtered` | `geographic_msgs/msg/GeoPointStamped` | (custom) | unused |
| `/ap/airspeed` | `ardupilot_msgs/msg/Airspeed` | (custom) | unused |
| `/ap/rc` | `ardupilot_msgs/msg/Rc` | (custom) | unused |
| `/ap/status` | `ardupilot_msgs/msg/Status` | (custom) | unused |
| `/ap/navsat`, `/ap/tf`, `/ap/tf_static` | `sensor_msgs/NavSatFix`, `tf2_msgs/TFMessage` | — | unused |

### QoS (`/ap/pose/filtered`, representative of the AP_DDS publishers)

```
Reliability: BEST_EFFORT
Durability:  VOLATILE
Liveliness:  AUTOMATIC
History (Depth): UNKNOWN
Node name: _CREATED_BY_BARE_DDS_APP_   (AP_DDS is a bare DDS participant)
```

The node's subscriptions must use a BEST_EFFORT/VOLATILE-compatible profile
(`rclpy.qos.qos_profile_sensor_data`), which `node.py` does.

## Finding 1 — IMU / specific-force topic EXISTS

`/ap/imu/experimental/data` (`sensor_msgs/msg/Imu`) publishes at ~126 Hz. This
is the accelerometer specific force the INDI accel loop needs. **Decision: the
bridge default `indi_accel=True` is valid**; the `--no-indi` PD+ff fallback is
therefore a deliberate option, not a forced degradation. The IMU body frame is
ROS FLU (REP-103) → `OffboardMission.on_imu` maps `linear_acceleration` to FRD
via `[a.x, -a.y, -a.z]`.

## Finding 2 — header-stamp epoch is system/Unix time, NOT ArduPilot boot ms

- `/ap/pose/filtered` `header.stamp` sample: `sec=1784517735 nanosec≈140e6`
  (≈ Unix epoch; matches the VM RTC 2026-07-20 03:22 UTC).
- `/ap/time` sample: `sec=1784517736` — same Unix epoch.
- `LOCAL_POSITION_NED.time_boot_ms` (MAVLink 5790): `95494` (≈ 95.5 s since
  boot).

The AP_DDS header stamps and `/ap/time` are wall/Unix time; the `.BIN` dataflash
and `LOCAL_POSITION_NED` are ArduPilot boot-relative. These are different clocks.
**The trajectory↔BIN join in Task 4 therefore uses `LOCAL_POSITION_NED.time_boot_ms`
(the S1 mechanism), not the DDS pose stamps** — exactly as the plan requires. The
node still derives trajectory time from the pose `header.stamp` deltas (monotone,
sim-time-consistent) for feedforward phasing; only the .BIN scoring alignment uses
boot ms.

## Finding 3 — AP_DDS command-topic set is too coarse for attitude+thrust

AP_DDS exposes only these **subscriber** (command-in) topics:

| Topic | Type | Sub count |
|-------|------|-----------|
| `/ap/cmd_gps_pose` | `ardupilot_msgs/msg/GlobalPosition` | 1 |
| `/ap/cmd_vel` | `geometry_msgs/msg/TwistStamped` | 1 |
| `/ap/joy` | `sensor_msgs/msg/Joy` | 1 |

There is **no attitude+body-rate+thrust command topic** in the AP_DDS graph — the
finest-grained command is a velocity twist. This is the documented justification
for routing the S2 command path over MAVLink `SET_ATTITUDE_TARGET` on port 5790
(design doc §T.1 pragmatic route) rather than through AP_DDS. Extending AP_DDS
with an attitude-target subscriber is deferred to S3+ (§T.3).

## Finding 4 — pose rate (~27.6 Hz) is below the assumed 50 Hz

The event-driven command rate equals the `/ap/pose/filtered` rate (~27.6 Hz in
this SITL config), not the 50 Hz the bridge filters are constructed with by
default (`rate_hz=50`). The Butter2 accel/thrust-state filters take `fs` at
construction, so a 27.6 Hz stream fed to a 50 Hz-designed filter shifts the
effective cutoff. For S2 (outer-loop logic + filter-tuning phase, latency-
dominated; absolute-number mismatch is an accepted known limitation) this is
acceptable, but Task 6's sitl-env should consider passing `rate_hz≈28` (or the
attribution doc should note the mismatch) so the filter cutoff is as designed.
Recorded here as the landing site for that tuning decision.
