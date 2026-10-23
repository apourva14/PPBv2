#!/opt/vio-tools/bin/python -I
"""Score a recorded VIO run (/vio/odom) against ground truth.

Reads <run_dir>/record (ROS 2 bag written during replay), writes
<run_dir>/vio_odom.tum and <run_dir>/evaluation.json, prints a summary, and
exits non-zero if the acceptance limits for the sequence are not met.

Metrics:
  APE  absolute translation error after aligning position + yaw only (the 4 DoF
       VIO cannot observe; roll/pitch errors stay visible, scale is not fitted).
       The SE(3)-aligned APE is reported too, for comparison with other tools.
  RPE  translation and heading drift per 10 m travelled.
  availability  init latency, output coverage, longest output gap.
  input health  /vio/status from vio_stream_monitor. If the IMU or camera
       streams were degraded during the replay (e.g. the machine could not keep
       up and messages were dropped), the run is INVALID: the estimator was not
       given the data, so its score says nothing about the estimator.

Exit codes: 0 pass, 1 estimator failed acceptance, 3 invalid run (inputs degraded).
"""

import argparse
import copy
import json
from pathlib import Path
import sys

from evo.core import metrics, sync
from evo.core.trajectory import PoseTrajectory3D
from evo.tools import file_interface
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import get_typestore, Stores
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ppb_vio_tools.metrics import (  # noqa: E402
    availability, check_acceptance, input_health, position_errors_after_yaw_alignment)

ODOM_TOPIC = '/vio/odom'
STATUS_TOPIC = '/vio/status'
# Ignore input diagnostics this long after the replay starts (window warm-up).
STATUS_WARMUP_S = 3.0
ASSOCIATION_MAX_DIFF_S = 0.02
RPE_DELTA_M = 10.0


def read_bag_span(bag):
    with AnyReader([Path(bag)], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
        return reader.start_time / 1e9, reader.end_time / 1e9


def read_odometry(bag):
    stamps, xyz, wxyz, frames = [], [], [], set()
    with AnyReader([Path(bag)], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
        conns = [c for c in reader.connections if c.topic == ODOM_TOPIC]
        if not conns:
            raise RuntimeError(f'{bag}: no {ODOM_TOPIC} recorded')
        for conn, _, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            xyz.append([p.x, p.y, p.z])
            wxyz.append([q.w, q.x, q.y, q.z])          # evo uses w-first quaternions
            frames.add((msg.header.frame_id, msg.child_frame_id))
    return np.array(stamps), np.array(xyz), np.array(wxyz), sorted(frames)


def read_status(bag):
    samples = []
    with AnyReader([Path(bag)], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
        conns = [c for c in reader.connections if c.topic == STATUS_TOPIC]
        for conn, _, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            for st in msg.status:
                level = st.level if isinstance(st.level, int) else int.from_bytes(st.level, 'little')
                samples.append((stamp, st.name, level))
    return samples


def rpe(ref, est, relation):
    metric = metrics.RPE(relation, delta=RPE_DELTA_M, delta_unit=metrics.Unit.meters,
                         all_pairs=False)
    metric.process_data((ref, est))
    return metric.get_statistic(metrics.StatisticsType.rmse)


def evaluate(run_dir, sensor_bag, gt_path):
    run_dir = Path(run_dir)
    stamps, xyz, wxyz, frames = read_odometry(run_dir / 'record')
    sensor_start, sensor_end = read_bag_span(sensor_bag)
    avail = availability(stamps, sensor_start, sensor_end)
    fractions, degraded = input_health(read_status(run_dir / 'record'),
                                       sensor_start + STATUS_WARMUP_S, sensor_end)

    est = PoseTrajectory3D(positions_xyz=xyz, orientations_quat_wxyz=wxyz, timestamps=stamps)
    file_interface.write_tum_trajectory_file(str(run_dir / 'vio_odom.tum'), est)

    ref = file_interface.read_tum_trajectory_file(str(gt_path))
    ref, est = sync.associate_trajectories(ref, est, max_diff=ASSOCIATION_MAX_DIFF_S)
    errors = position_errors_after_yaw_alignment(ref.positions_xyz, est.positions_xyz)

    est_se3 = copy.deepcopy(est)
    est_se3.align(ref, correct_scale=False)
    ape_se3 = metrics.APE(metrics.PoseRelation.translation_part)
    ape_se3.process_data((ref, est_se3))

    result = {
        'odom_frames': [list(f) for f in frames],
        'odom_messages': int(stamps.size),
        'matched_poses': int(est.num_poses),
        'trajectory_length_m': float(ref.path_length),
        'ape_rmse_m': float(np.sqrt(np.mean(errors ** 2))),
        'ape_max_m': float(errors.max()),
        'ape_se3_rmse_m': float(ape_se3.get_statistic(metrics.StatisticsType.rmse)),
        'availability': avail.as_dict(),
        'input_degraded_fraction': {k: round(v, 4) for k, v in sorted(fractions.items())},
        'input_degraded': degraded,
    }
    if ref.path_length > 2 * RPE_DELTA_M:
        result['rpe_translation_rmse_m_per_10m'] = float(
            rpe(ref, est, metrics.PoseRelation.translation_part))
        result['rpe_rotation_rmse_deg_per_10m'] = float(
            rpe(ref, est, metrics.PoseRelation.rotation_angle_deg))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--run-dir', required=True)
    parser.add_argument('--sensor-bag', required=True, help='bag that was replayed')
    parser.add_argument('--gt', required=True, help='ground truth, TUM format')
    parser.add_argument('--sequence', required=True, help='key in the acceptance file')
    parser.add_argument('--acceptance', required=True, help='acceptance limits YAML')
    args = parser.parse_args()

    result = evaluate(args.run_dir, args.sensor_bag, args.gt)
    limits_all = yaml.safe_load(Path(args.acceptance).read_text())
    limits = limits_all.get(args.sequence, limits_all['default'])
    failures = check_acceptance(result, limits)
    invalid = result['input_degraded']
    result.update({'sequence': args.sequence, 'limits': limits, 'failures': failures,
                   'valid': not invalid, 'passed': not failures and not invalid})
    (Path(args.run_dir) / 'evaluation.json').write_text(json.dumps(result, indent=2))

    a = result['availability']
    print(f"sequence            {args.sequence}")
    print(f"odom frames         {result['odom_frames']}")
    print(f"matched poses       {result['matched_poses']} / {result['odom_messages']} outputs")
    print(f"trajectory length   {result['trajectory_length_m']:.1f} m")
    print(f"APE RMSE / max      {result['ape_rmse_m']:.3f} m / {result['ape_max_m']:.3f} m "
          f"(position+yaw aligned; SE(3)-aligned RMSE {result['ape_se3_rmse_m']:.3f} m)")
    if 'rpe_translation_rmse_m_per_10m' in result:
        print(f"RPE per 10 m        {result['rpe_translation_rmse_m_per_10m']:.3f} m, "
              f"{result['rpe_rotation_rmse_deg_per_10m']:.2f} deg")
    print(f"init latency        {a['init_latency_s']:.1f} s")
    print(f"coverage            {a['coverage']:.3f} (longest gap {a['longest_gap_s']:.2f} s, "
          f"{a['output_rate_hz']:.1f} Hz)")
    worst = max(result['input_degraded_fraction'].items(), key=lambda kv: kv[1],
                default=('none', 1.0))
    print(f"input health        worst: {worst[0]} degraded {worst[1]:.1%} of the time")
    if invalid:
        print('RESULT              INVALID: sensor inputs were degraded during replay '
              '(dropped/late messages); rerun on a less loaded machine or with --rate lower')
        return 3
    print('RESULT              ' + ('PASS' if not failures else 'FAIL: ' + '; '.join(failures)))
    return 0 if not failures else 1


if __name__ == '__main__':
    sys.exit(main())
