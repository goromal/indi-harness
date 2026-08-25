"""Thin MAVLink flight helpers shared by the SITL frame/sign gates.

Factored out of the (previously duplicated) inline flight code in the anixpkgs
sitl-envs test scripts (arm-probe / drag-rejection frame gate): connect with a
valid-heartbeat guard, data-stream request, GPS/EKF warm-up, arm-through-prearm
retry, a drain-the-buffer freshest-LOCAL_POSITION_NED read, guided takeoff with
altitude-settle, and a held NED velocity move. Deliberately procedural (talks to
a pymavlink connection directly); the richer battery runners live in baseline*.

The custom-controller LOW->HIGH engage helper is NOT here -- it lives in
baseline_cc.engage_custom_controller; import it from there.
"""
import time
from pymavlink import mavutil


def connect(url="tcp:127.0.0.1:5790", timeout=180.0):
    """Open a MAVLink connection and wait for a heartbeat that carries a nonzero
    autopilot system id. A bare wait_heartbeat can return with target_system=0
    (timeout, or a heartbeat from a non-autopilot component), after which every
    command is addressed to system 0 and arming silently never takes. Returns the
    connection (with target_system possibly still 0 if none arrived in time; the
    caller should check)."""
    m = mavutil.mavlink_connection(url)
    t0 = time.time()
    while time.time() - t0 < timeout:
        m.wait_heartbeat(timeout=10)
        if m.target_system != 0:
            break
    return m


def request_data_stream(m, rate=5):
    """Ask the autopilot to stream ALL data-stream groups at `rate` Hz."""
    m.mav.request_data_stream_send(m.target_system, m.target_component,
                                   mavutil.mavlink.MAV_DATA_STREAM_ALL, rate, 1)


def wait_gps(m, timeout=45.0, min_settle=15.0):
    """Brief GPS/EKF settle. Prearm (the arm loop's retry) is the real readiness
    gate, so this is only a short warm-up, not a hard wait on a fix_type report
    (which routes inconsistently over the router port). Returns True if a 3D fix
    was ever seen. Waits until at least `min_settle` s have elapsed after the
    first fix, or `timeout` s total."""
    t0 = time.time()
    gps_ok = False
    while time.time() - t0 < timeout:
        g = m.recv_match(type="GPS_RAW_INT", blocking=True, timeout=2)
        if g and g.fix_type >= 3:
            gps_ok = True
        if gps_ok and time.time() - t0 > min_settle:
            break
    return gps_ok


def arm_with_retry(m, timeout=120.0, retry_s=2.0, verbose=False):
    """Arm, retrying through prearm: send the arm command, poll a HEARTBEAT for
    the SAFETY_ARMED flag, and retry until it takes or `timeout` elapses. When
    `verbose`, print any STATUSTEXT seen along the way (the concrete prearm-reject
    diagnostic). Returns True if armed."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                1, 0, 0, 0, 0, 0, 0)
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            return True
        if verbose:
            st = m.recv_match(type="STATUSTEXT", blocking=False)
            if st:
                print(f"[arm] STATUSTEXT: {st.text}", flush=True)
        time.sleep(retry_s)
    return False


def disarm(m, force=False):
    """Disarm (force=True sends the 21196 magic to force-disarm even in air)."""
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                            0, 21196 if force else 0, 0, 0, 0, 0, 0)


def get_local_pos(m, timeout=3.0):
    """Return the FRESHEST LOCAL_POSITION_NED as (x, y, z), or None.

    We stream at high rate and send setpoints in tight loops without reading, so
    the receive buffer backs up; a single blocking recv_match returns the OLDEST
    queued sample, which lags the true position by many seconds and makes a real
    move look like zero displacement. Drain everything pending, then return the
    last one (falling back to one blocking read if the buffer was momentarily
    empty)."""
    latest = None
    while True:
        msg = m.recv_match(type="LOCAL_POSITION_NED", blocking=False)
        if msg is None:
            break
        latest = msg
    if latest is None:
        latest = m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=timeout)
    return None if latest is None else (latest.x, latest.y, latest.z)


def takeoff(m, alt, timeout=150.0, settle_window=5.0, alt_tol=2.5, vz_tol=0.4):
    """Command a GUIDED takeoff to `alt` m and WAIT FOR THE ALTITUDE TO SETTLE.

    A high thrust-to-weight backend overshoots well past the target before the
    altitude loop damps it back; commanding horizontal moves during that climb is
    ignored. Waits until the vehicle is near `alt` with small vertical speed for a
    sustained `settle_window` -- a bounded, damped transient, not a runaway.
    Returns the settled (x, y, z) LOCAL_NED, or None if it never settled."""
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                            0, 0, 0, 0, 0, 0, alt)
    t0 = time.time()
    settled_since = None
    while time.time() - t0 < timeout:
        msg = m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=2)
        if msg is None:
            continue
        near = abs(-msg.z - alt) < alt_tol and abs(msg.vz) < vz_tol
        if near:
            settled_since = settled_since or time.time()
            if time.time() - settled_since > settle_window:
                return (msg.x, msg.y, msg.z)
        else:
            settled_since = None
    return None


def guided_move_ned(m, north, east, down, label="MOVE", dur=12.0):
    """Command a steady LOCAL_NED velocity (north, east, down) m/s and hold it
    for `dur` s, then measure net displacement. Velocity control is a cleaner
    frame/sign probe than an absolute position target (interpreted in the
    EKF-origin frame and slow to converge from a standstill). Returns (delta, end)
    where delta = (dN, dE, dD) and end is the freshest LOCAL_NED after the hold."""
    base = get_local_pos(m)
    t0 = time.time()
    while time.time() - t0 < dur:
        m.mav.set_position_target_local_ned_send(
            0, m.target_system, m.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,  # velocity only
            0, 0, 0, north, east, down, 0, 0, 0, 0, 0)
        time.sleep(0.1)
    end = get_local_pos(m)
    d = (end[0] - base[0], end[1] - base[1], end[2] - base[2])
    print(f"[gate] {label} base={base} end={end} "
          f"delta=(dN={d[0]:.2f},dE={d[1]:.2f},dD={d[2]:.2f})", flush=True)
    return d, end
