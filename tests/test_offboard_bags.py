import numpy as np
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore
from indi_harness.offboard.bags import read_topic, latency_report

TS = get_typestore(Stores.ROS2_JAZZY)


def write_bag(tmp_path, topic="/ap/pose/filtered", n=100, dt_ns=20_000_000):
    bag = tmp_path / "testbag"
    Pose = TS.types["geometry_msgs/msg/PoseStamped"]
    Header = TS.types["std_msgs/msg/Header"]
    Time = TS.types["builtin_interfaces/msg/Time"]
    P = TS.types["geometry_msgs/msg/Pose"]
    Pt = TS.types["geometry_msgs/msg/Point"]
    Q = TS.types["geometry_msgs/msg/Quaternion"]
    # rosbags 0.11 requires an explicit bag version keyword on the Writer.
    with Writer(bag, version=9) as w:
        conn = w.add_connection(topic, Pose.__msgtype__, typestore=TS)
        for k in range(n):
            t = 1_000_000_000 + k * dt_ns
            msg = Pose(header=Header(stamp=Time(sec=t // 10**9,
                                                nanosec=t % 10**9),
                                     frame_id="map"),
                       pose=P(position=Pt(x=0.0, y=0.0, z=10.0),
                              orientation=Q(x=0.0, y=0.0, z=0.0, w=1.0)))
            w.write(conn, t, TS.serialize_cdr(msg, Pose.__msgtype__))
    return bag


def test_read_topic(tmp_path):
    bag = write_bag(tmp_path)
    rows = read_topic(bag, "/ap/pose/filtered")
    assert len(rows) == 100
    t0, m0 = rows[0]
    assert m0.pose.position.z == 10.0


def test_latency_report(tmp_path):
    bag = write_bag(tmp_path)
    rep = latency_report(bag)
    st = rep["/ap/pose/filtered"]
    assert st["n"] == 100
    assert abs(st["inter_arrival_p50_ms"] - 20.0) < 1.0
