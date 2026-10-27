#!/opt/vio-tools/bin/python -I
"""Write a stressed copy of a /vio/* contract bag for the VIO bench (seeded, repeatable).

Stressing the input bag, rather than the running system, keeps every stress
run deterministic: the same bag through the offline bench gives the same
result, and two frontends see exactly the same corrupted data.

    stress_bag.py IN_BAG OUT_BAG [--drop-frames P] [--blackout T0:T1] [--jitter-ms S]
                  [--imu-drop P] [--start S] [--seed N]

  --drop-frames P   drop each stereo pair with probability P (both cameras, as a
                    driver that loses a frame does; camera_info goes with it)
  --blackout T0:T1  drop every camera frame between T0 and T1 s after the bag start
                    (lens covered / sun glare / tracking loss)
  --jitter-ms S     add N(0, S) ms to each stereo pair's stamp (same for both
                    cameras, so pairing survives); stamps stay increasing
  --imu-drop P      drop each IMU sample with probability P
  --start S         drop everything before S s after the bag start (initialize
                    from a different point, e.g. at speed, mid-row)

Writes OUT_BAG and OUT_BAG.stress.json (what was changed, for the run record).
"""

import argparse
import json
from pathlib import Path
import shutil
import sys

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import get_typestore, Stores

HUMBLE_BAG_VERSION = 8
CAMERA_TOPICS = ('/vio/cam0/image_raw', '/vio/cam1/image_raw',
                 '/vio/cam0/camera_info', '/vio/cam1/camera_info')
IMU_TOPIC = '/vio/imu'


def parse_blackout(text):
    a, b = (float(v) for v in text.split(':'))
    if b <= a:
        raise argparse.ArgumentTypeError('blackout must be T0:T1 with T1 > T0')
    return a, b


def stress(in_bag, out_bag, drop_frames=0.0, blackout=None, jitter_ms=0.0, imu_drop=0.0,
           start_s=0.0, seed=0):
    rng = np.random.default_rng(seed)
    in_bag, out_bag = Path(in_bag), Path(out_bag)
    if out_bag.exists():
        raise SystemExit(f'output exists: {out_bag}')
    ts = get_typestore(Stores.ROS2_HUMBLE)
    decisions = {}          # camera stamp_ns -> (keep, new stamp_ns), shared by all camera topics
    last_cam_stamp = -1
    counts = {'camera_pairs_in': 0, 'camera_pairs_dropped': 0, 'camera_pairs_blacked_out': 0,
              'imu_in': 0, 'imu_dropped': 0, 'messages_before_start': 0, 'stamps_jittered': 0}
    partial = out_bag.with_name(out_bag.name + '.partial')
    shutil.rmtree(partial, ignore_errors=True)
    try:
        with AnyReader([in_bag]) as reader, Writer(partial, version=HUMBLE_BAG_VERSION,
                                                   storage_plugin=StoragePlugin.MCAP) as writer:
            t0 = reader.start_time
            out_conns = {c.id: writer.add_connection(c.topic, c.msgtype, typestore=ts)
                         for c in reader.connections}
            for conn, t, raw in reader.messages():
                rel = (t - t0) / 1e9
                if rel < start_s:
                    counts['messages_before_start'] += 1
                    continue
                if conn.topic == IMU_TOPIC:
                    counts['imu_in'] += 1
                    if imu_drop and rng.random() < imu_drop:
                        counts['imu_dropped'] += 1
                        continue
                    writer.write(out_conns[conn.id], t, raw)
                    continue
                if conn.topic not in CAMERA_TOPICS:
                    writer.write(out_conns[conn.id], t, raw)
                    continue
                msg = reader.deserialize(raw, conn.msgtype)
                stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
                if stamp not in decisions:
                    counts['camera_pairs_in'] += 1
                    keep = True
                    if blackout and blackout[0] <= (stamp - t0) / 1e9 < blackout[1]:
                        keep = False
                        counts['camera_pairs_blacked_out'] += 1
                    elif drop_frames and rng.random() < drop_frames:
                        keep = False
                        counts['camera_pairs_dropped'] += 1
                    new = stamp
                    if keep and jitter_ms:
                        new = stamp + int(rng.normal(0.0, jitter_ms) * 1e6)
                        new = max(new, last_cam_stamp + 1_000_000)      # stay increasing (>= 1 ms)
                        counts['stamps_jittered'] += 1
                    if keep:
                        last_cam_stamp = new
                    decisions[stamp] = (keep, new)
                keep, new = decisions[stamp]
                if not keep:
                    continue
                if new != stamp:
                    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(new, 10**9)
                    raw = ts.serialize_cdr(msg, conn.msgtype)
                writer.write(out_conns[conn.id], new, raw)
    except BaseException:
        shutil.rmtree(partial, ignore_errors=True)
        raise
    partial.rename(out_bag)
    report = {'input': str(in_bag), 'output': str(out_bag), 'seed': seed,
              'drop_frames': drop_frames, 'blackout_s': list(blackout) if blackout else None,
              'jitter_ms': jitter_ms, 'imu_drop': imu_drop, 'start_s': start_s, 'counts': counts}
    Path(str(out_bag) + '.stress.json').write_text(json.dumps(report, indent=2))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('in_bag')
    parser.add_argument('out_bag')
    parser.add_argument('--drop-frames', type=float, default=0.0)
    parser.add_argument('--blackout', type=parse_blackout)
    parser.add_argument('--jitter-ms', type=float, default=0.0)
    parser.add_argument('--imu-drop', type=float, default=0.0)
    parser.add_argument('--start', type=float, default=0.0)
    parser.add_argument('--seed', type=int, default=0)
    args = parser.parse_args()
    report = stress(args.in_bag, args.out_bag, args.drop_frames, args.blackout, args.jitter_ms,
                    args.imu_drop, args.start, args.seed)
    print(json.dumps(report, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
