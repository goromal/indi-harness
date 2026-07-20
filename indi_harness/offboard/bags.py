"""rosbag2 reader (mcap or sqlite3) + latency stats (design doc §L: the bag
records what the graph saw; this file turns it into evidence). Pure-python
via `rosbags` — usable on host and in the VM, no ROS env required.

Usage: python3 -m indi_harness.offboard.bags <bag_dir> [--json out.json]
"""
import argparse
import glob
import json
import os
import sqlite3
import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

_TS = get_typestore(Stores.ROS2_JAZZY)


def _read_sqlite3(bag_dir, topic):
    """Direct sqlite3 read, bypassing rosbags' rihs01 type-hash validation.

    rosbags 0.11 asserts the bag's per-type rihs01 digest matches its own
    typestore hash; a real `ros2 bag record` (+reindex) bag carries ROS-native
    digests that fail that check for some builtin types. The .db3 payload is
    plain CDR, so we deserialize it ourselves against the jazzy typestore.
    """
    db3 = sorted(glob.glob(os.path.join(str(bag_dir), "*.db3")))
    if not db3:
        raise FileNotFoundError(f"no .db3 in {bag_dir}")
    con = sqlite3.connect(db3[0])
    try:
        tid_type = {name: (tid, ttype) for tid, name, ttype in
                    con.execute("select id, name, type from topics")}
        if topic not in tid_type:
            return []
        tid, ttype = tid_type[topic]
        return [(int(ts), _TS.deserialize_cdr(bytes(data), ttype))
                for ts, data in con.execute(
                    "select timestamp, data from messages "
                    "where topic_id=? order by timestamp", (tid,))]
    finally:
        con.close()


def read_topic(bag_dir, topic):
    """(log_time_ns, msg) rows for a topic. Tries the rosbags Reader (mcap or
    sqlite3); falls back to a direct sqlite3 read when the Reader rejects a
    real ROS2 bag's type digests."""
    try:
        rows = []
        with Reader(str(bag_dir)) as r:
            conns = [c for c in r.connections if c.topic == topic]
            for conn, t_ns, raw in r.messages(connections=conns):
                rows.append((t_ns, _TS.deserialize_cdr(raw, conn.msgtype)))
        return rows
    except Exception:
        # Reader can raise ReaderError, AssertionError, FileNotFoundError, ...
        if glob.glob(os.path.join(str(bag_dir), "*.db3")):
            return _read_sqlite3(bag_dir, topic)
        raise


def latency_report(bag_dir, topics=("/ap/pose/filtered", "/indi/cmd_attitude")):
    rep = {}
    for topic in topics:
        rows = read_topic(bag_dir, topic)
        if not rows:
            rep[topic] = {"n": 0}
            continue
        t = np.array([r[0] for r in rows], float)
        dt_ms = np.diff(t) / 1e6
        rep[topic] = {
            "n": len(rows),
            "duration_s": float((t[-1] - t[0]) / 1e9),
            "rate_hz": float((len(rows) - 1) / max((t[-1] - t[0]) / 1e9, 1e-9)),
            "inter_arrival_p50_ms": float(np.percentile(dt_ms, 50)),
            "inter_arrival_p95_ms": float(np.percentile(dt_ms, 95)),
            "inter_arrival_max_ms": float(dt_ms.max()),
        }
    return rep


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag")
    ap.add_argument("--json", default=None)
    args = ap.parse_args()
    rep = latency_report(args.bag)
    print(json.dumps(rep, indent=1))
    if args.json:
        with open(args.json, "w") as f:
            json.dump(rep, f, indent=1)


if __name__ == "__main__":
    main()
