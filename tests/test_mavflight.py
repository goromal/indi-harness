from types import SimpleNamespace

from pymavlink import mavutil

from indi_harness.sitl import mavflight


class DelayedDisarmConnection:
    target_system = 1
    target_component = 1

    def __init__(self, disarm_after):
        self.mav = self
        self.disarm_after = disarm_after
        self.disarm_commands = 0

    def command_long_send(self, *args):
        if args[2] == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and args[4] == 0:
            self.disarm_commands += 1

    def recv_match(self, **kwargs):
        armed = self.disarm_commands < self.disarm_after
        flag = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0
        return SimpleNamespace(base_mode=flag)


def test_disarm_with_retry_waits_for_disarmed_heartbeat():
    connection = DelayedDisarmConnection(disarm_after=3)
    assert mavflight.disarm_with_retry(connection, timeout=1.0, retry_s=0.0,
                                       force=True)
    assert connection.disarm_commands == 3
