#!/opt/vio-tools/bin/python -I
"""Score an estimated trajectory against ground truth.

The estimate is an odometry topic in a ROS 2 bag (default: /vio/odom in
<run_dir>/record, as recorded by scripts/run_replay.sh) or a TUM file. Ground
truth and its conventions come from the dataset.yaml written by a dataset
loader (prepare_asl.py, prepare_rosario_v1.py), or from --gt for a TUM file
that tracks base_link with full orientation (the EuRoC case).

Writes <run_dir>/estimate.tum and <run_dir>/evaluation.json, prints a summary,
and, with --acceptance, exits non-zero if the limits for the sequence are not met.

Metrics:
  ATE  absolute translation error after aligning position + yaw only (the 4 DoF
       VIO cannot observe; roll/pitch errors stay visible, scale is not fitted),
       at the point the ground truth tracks (e.g. Rosario's GPS antenna). Also
       horizontal-only and SE(3)-aligned variants for comparison.
  RTE  translation error of consecutive 10 m segments (positions only, so it
       works for position-only ground truth); rotation error per 10 m only when
       the ground truth has orientation.
  NEES position NEES from the odometry covariance (stub: reported, not gated).
  availability  init latency, output coverage, longest output gap.
  input health  /vio/status from vio_stream_monitor (live replays of /vio/odom).
       If the IMU or camera streams were degraded during the replay, the run is
       INVALID: the estimator was not given the data, so its score says nothing
       about it. --offline runs (bench: the estimator read the bag directly)
       have no input stream to monitor.
  divergence / health gate  when the estimate's speed departs from the ground
       truth's (see metrics.divergence_onset), whether the 'ppb_vio: vio health'
       gate latched ERROR and how late, and the ATE of the healthy part before
       the gate fired (what a fusion layer that drops unhealthy VIO receives).

Estimates are interpolated at ground-truth stamps, never across output gaps
longer than --max-bracket.

Exit codes: 0 pass (or no acceptance file), 1 failed acceptance, 3 invalid run.
"""

import argparse
import copy
import json
from pathlib import Path
import sys

from evo.core import metrics
from evo.core.trajectory import PoseTrajectory3D
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import get_typestore, Stores
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ppb_vio_tools.metrics import (  # noqa: E402
    align_position_yaw, availability, check_acceptance, divergence_onset, input_health,
    position_nees, relative_translation_errors)
from ppb_vio_tools.trajectory import (  # noqa: E402
    associate, read_tum, reference_point, Trajectory, write_tum)

ODOM_TOPIC = '/vio/odom'
STATUS_TOPIC = '/vio/status'
# Ignore input diagnostics this long after the replay starts (window warm-up).
STATUS_WARMUP_S = 3.0
RTE_DELTA_M = 10.0


def open_bag(bag):
    return AnyReader([Path(bag)], default_typestore=get_typestore(Stores.ROS2_HUMBLE))


def read_bag_span(bag):
    with open_bag(bag) as reader:
        return reader.start_time / 1e9, reader.end_time / 1e9


def read_odometry(bag, topic):
    """Return (Trajectory with covariances, sorted list of (frame_id, child_frame_id))."""
    stamps, xyz, xyzw, covs, frames = [], [], [], [], set()
    with open_bag(bag) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            raise RuntimeError(f'{bag}: no {topic} recorded')
        for conn, _, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            xyz.append([p.x, p.y, p.z])
            xyzw.append([q.x, q.y, q.z, q.w])
            covs.append(np.asarray(msg.pose.covariance, dtype=float).reshape(6, 6))
            frames.add((msg.header.frame_id, msg.child_frame_id))
    # A recorder may write messages that arrive together out of stamp order; score
    # in stamp order and drop exact duplicates (the count is reported).
    stamps = np.array(stamps)
    order = np.argsort(stamps, kind='stable')
    keep = order[np.concatenate([[True], np.diff(stamps[order]) > 0])]
    reordered = int(np.sum(np.diff(stamps) <= 0))
    covs = np.array(covs)[keep]
    traj = Trajectory(stamps[keep], np.array(xyz)[keep], np.array(xyzw)[keep],
                      covs if np.any(covs) else None)
    return traj, sorted(frames), {'out_of_order': reordered,
                                  'duplicates_dropped': int(stamps.size - keep.size)}


def read_status(bag):
    samples = []
    with open_bag(bag) as reader:
        conns = [c for c in reader.connections if c.topic == STATUS_TOPIC]
        if not conns:           # messages(connections=[]) would yield every message
            return samples
        for conn, _, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            for st in msg.status:
                level = st.level if isinstance(st.level, int) else int.from_bytes(st.level, 'little')
                samples.append((stamp, st.name, level))
    return samples


HEALTH_STATUS = 'ppb_vio: vio health'


def compute_budget(timing_csv, frame_period_s):
    """Per-update processing time of an offline bench run (estimator/timing.csv) against
    the camera frame period: the estimator keeps up live only if updates fit the period.
    Measured on the machine that ran the bench (the Orin's cores are slower than a
    laptop's: re-run the bench on the Orin before trusting the margin there)."""
    t = np.loadtxt(timing_csv, delimiter=',', skiprows=1, ndmin=2)
    if t.size == 0:
        return None
    p = t[:, 1]
    return {'updates': int(p.size), 'frame_period_ms': 1000 * frame_period_s,
            'p50_ms': float(1000 * np.median(p)), 'p95_ms': float(1000 * np.percentile(p, 95)),
            'max_ms': float(1000 * p.max()),
            'fraction_over_period': float(np.mean(p > frame_period_s))}


def gate_error_time(status_samples, latched_at=()):
    """When the vio health gate latched ERROR, in data time.

    The gate reports the data stamp it latched at (status value diverged_at_s);
    status headers carry the node clock, which differs from data time on the robot
    and in replays of old recordings, so the header stamp is only a fallback.
    """
    if latched_at:
        return min(latched_at)
    times = [stamp for stamp, name, level in status_samples if name == HEALTH_STATUS and level == 2]
    return min(times) if times else None


def read_gate_latch(bag):
    """diverged_at_s values reported by the vio health gate (data time)."""
    out = []
    with open_bag(bag) as reader:
        conns = [c for c in reader.connections if c.topic == STATUS_TOPIC]
        if not conns:
            return out
        for conn, _, raw in reader.messages(connections=conns):
            for st in reader.deserialize(raw, conn.msgtype).status:
                if st.name == HEALTH_STATUS:
                    out += [float(kv.value) for kv in st.values if kv.key == 'diverged_at_s']
    return out


def load_dataset(args):
    """Ground-truth conventions: from dataset.yaml, or EuRoC-style defaults for --gt."""
    if args.dataset:
        path = Path(args.dataset)
        d = yaml.safe_load(path.read_text())
        return {
            'sequence': d['sequence'],
            # an explicit --sensor-bag (e.g. a stress variant) overrides the dataset's bag
            'sensor_bag': args.sensor_bag or str(path.parent / d['sensor_bag']),
            'ground_truth': str(path.parent / d['ground_truth']),
            'has_orientation': bool(d['ground_truth_has_orientation']),
            'point_in_base': [float(v) for v in d['ground_truth_point_in_base']],
        }
    if not (args.gt and args.sensor_bag and args.sequence):
        raise SystemExit('give --dataset, or all of --gt, --sensor-bag and --sequence')
    return {'sequence': args.sequence, 'sensor_bag': args.sensor_bag, 'ground_truth': args.gt,
            'has_orientation': True, 'point_in_base': [0.0, 0.0, 0.0]}


def evo_trajectory(stamps, xyz, quats_xyzw):
    return PoseTrajectory3D(positions_xyz=xyz, orientations_quat_wxyz=np.roll(quats_xyzw, 1, axis=1),
                            timestamps=stamps)


def align_to_ground_truth(est, ds, max_bracket_s):
    """Estimate at the ground-truth point and stamps, position+yaw aligned to it.

    Returns (ref Trajectory, matched ref indices, estimate interpolated at them,
    its reference-point positions and covariances, yaw rotation, aligned positions).
    """
    ref = read_tum(ds['ground_truth'])
    idx, est_at_ref = associate(ref.stamps, est, max_bracket_s)
    if idx.size < 2:
        raise RuntimeError(f'only {idx.size} estimates overlap the ground truth')
    est_xyz, est_cov = reference_point(est_at_ref, ds['point_in_base'])
    rot, t = align_position_yaw(ref.positions[idx], est_xyz)
    return ref, idx, est_at_ref, est_xyz, est_cov, rot, est_xyz @ rot.T + t


def evaluate(est, ds, max_bracket_s):
    ref, idx, est_at_ref, est_xyz, est_cov, rot, aligned = align_to_ground_truth(
        est, ds, max_bracket_s)
    ref_xyz = ref.positions[idx]
    err = aligned - ref_xyz
    norms = np.linalg.norm(err, axis=1)
    horiz = np.linalg.norm(err[:, :2], axis=1)

    ref_evo = evo_trajectory(ref.stamps[idx], ref_xyz, ref.quats_xyzw[idx])
    est_evo = evo_trajectory(est_at_ref.stamps, est_xyz, est_at_ref.quats_xyzw)
    est_se3 = copy.deepcopy(est_evo)
    est_se3.align(ref_evo, correct_scale=False)
    ape_se3 = metrics.APE(metrics.PoseRelation.translation_part)
    ape_se3.process_data((ref_evo, est_se3))

    path_length = float(np.sum(np.linalg.norm(np.diff(ref_xyz, axis=0), axis=1)))
    result = {
        'matched_poses': int(idx.size),
        'ground_truth_poses': int(len(ref)),
        'trajectory_length_m': path_length,
        'ape_rmse_m': float(np.sqrt(np.mean(norms ** 2))),
        'ape_max_m': float(norms.max()),
        'ape_horizontal_rmse_m': float(np.sqrt(np.mean(horiz ** 2))),
        'ape_se3_rmse_m': float(ape_se3.get_statistic(metrics.StatisticsType.rmse)),
        'alignment_yaw_deg': float(np.degrees(np.arctan2(rot[1, 0], rot[0, 0]))),
    }
    rte = relative_translation_errors(ref_xyz, aligned, RTE_DELTA_M)
    if rte.size:
        result['rte_translation_rmse_m_per_10m'] = float(np.sqrt(np.mean(rte ** 2)))
        result['rte_segments'] = int(rte.size)
    if ds['has_orientation'] and path_length > 2 * RTE_DELTA_M:
        rpe = metrics.RPE(metrics.PoseRelation.rotation_angle_deg, delta=RTE_DELTA_M,
                          delta_unit=metrics.Unit.meters, all_pairs=False)
        rpe.process_data((ref_evo, est_evo))
        result['rpe_rotation_rmse_deg_per_10m'] = float(
            rpe.get_statistic(metrics.StatisticsType.rmse))
    onset = divergence_onset(ref.stamps[idx], ref_xyz, est_xyz)
    result['divergence_onset_s'] = onset
    if est_cov is None:
        result['nees_position'] = {'status': 'unavailable', 'reason': 'estimate has no covariance'}
    else:
        # Covariance is in the estimator world frame; the error is in the aligned frame.
        result['nees_position'] = position_nees(err, rot @ est_cov[:, :3, :3] @ rot.T)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--run-dir', required=True, help='where results are written')
    src = parser.add_mutually_exclusive_group()
    src.add_argument('--estimate-bag', help='bag with the odometry topic (default: <run-dir>/record)')
    src.add_argument('--estimate-tum', help='TUM file instead of a bag (no covariance)')
    parser.add_argument('--topic', default=ODOM_TOPIC, help=f'odometry topic (default {ODOM_TOPIC})')
    parser.add_argument('--dataset', help='dataset.yaml written by a dataset loader')
    parser.add_argument('--gt', help='ground truth, TUM format, tracking base_link (no --dataset)')
    parser.add_argument('--sensor-bag', help='bag that was replayed (default: the dataset\'s)')
    parser.add_argument('--sequence', help='key in the acceptance file (no --dataset)')
    parser.add_argument('--acceptance', help='acceptance limits YAML (omit to only report)')
    parser.add_argument('--offline', action='store_true',
                        help='bench run: the estimator read the bag directly (no live input stream)')
    parser.add_argument('--max-bracket', type=float, default=0.25,
                        help='longest estimate gap interpolated across, s (default 0.25)')
    args = parser.parse_args()

    ds = load_dataset(args)
    run_dir = Path(args.run_dir)
    run_dir.mkdir(parents=True, exist_ok=True)
    if args.estimate_tum:
        est, frames, source = read_tum(args.estimate_tum), [], args.estimate_tum
        ordering = {}
    else:
        bag = args.estimate_bag or run_dir / 'record'
        est, frames, ordering = read_odometry(bag, args.topic)
        source = f'{bag}:{args.topic}'
    write_tum(run_dir / 'estimate.tum', est)

    result = evaluate(est, ds, args.max_bracket)
    sensor_start, sensor_end = read_bag_span(ds['sensor_bag'])
    avail = availability(est.stamps, sensor_start, sensor_end)
    statuses = [] if args.estimate_tum else read_status(args.estimate_bag or run_dir / 'record')
    monitored = args.topic == ODOM_TOPIC and not args.estimate_tum and not args.offline
    if monitored:
        fractions, degraded = input_health(statuses, sensor_start + STATUS_WARMUP_S, sensor_end)
    else:
        fractions, degraded = {}, False

    # Divergence vs. the health gate, times relative to the start of sensor data.
    latched = [] if args.estimate_tum else read_gate_latch(args.estimate_bag or run_dir / 'record')
    onset, gate = result['divergence_onset_s'], gate_error_time(statuses, latched)
    rel = lambda t: None if t is None else round(t - sensor_start, 2)  # noqa: E731
    health = {'divergence_onset_s': rel(onset), 'gate_error_s': rel(gate),
              'gate_reported': any(name == HEALTH_STATUS for _, name, _ in statuses),
              'caught': None if onset is None else (gate is not None),
              'detection_latency_s': None if onset is None or gate is None
              else round(gate - onset, 2),
              'false_alarm': onset is None and gate is not None}
    if gate is not None:
        healthy = est.stamps < gate
        if np.count_nonzero(healthy) >= 2:
            try:
                h = evaluate(Trajectory(est.stamps[healthy], est.positions[healthy],
                                        est.quats_xyzw[healthy],
                                        None if est.covariances is None
                                        else est.covariances[healthy]), ds, args.max_bracket)
                health.update({'healthy_ape_rmse_m': h['ape_rmse_m'],
                               'healthy_nees_position_mean': h['nees_position'].get('mean'),
                               'healthy_matched_poses': h['matched_poses'],
                               'healthy_trajectory_length_m': h['trajectory_length_m']})
            except RuntimeError:
                pass
    del result['divergence_onset_s']

    timing = run_dir / 'estimator' / 'timing.csv'
    if timing.is_file():
        stamps = np.loadtxt(timing, delimiter=',', skiprows=1, ndmin=2)[:, 0]
        period = float(np.median(np.diff(stamps))) if stamps.size > 1 else float('nan')
        result['compute'] = compute_budget(timing, period)
    result.update({
        'sequence': ds['sequence'],
        'estimate_source': source,
        'odom_frames': [list(f) for f in frames],
        'odom_messages': len(est),
        'odom_record_ordering': ordering,
        'ground_truth': {k: ds[k] for k in ('ground_truth', 'has_orientation', 'point_in_base')},
        'availability': avail.as_dict(),
        'health_gate': health,
        'input_monitored': monitored,
        'input_degraded_fraction': {k: round(v, 4) for k, v in sorted(fractions.items())},
        'input_degraded': degraded,
    })

    failures, limits = [], None
    if args.acceptance:
        limits_all = yaml.safe_load(Path(args.acceptance).read_text())
        limits = limits_all.get(ds['sequence'], limits_all['default'])
        failures = check_acceptance(result, limits)
    result.update({'limits': limits, 'failures': failures, 'valid': not degraded,
                   'passed': (not failures and not degraded) if limits else None})
    (run_dir / 'evaluation.json').write_text(json.dumps(result, indent=2))

    a, nees = result['availability'], result['nees_position']
    print(f"sequence            {ds['sequence']}")
    print(f"estimate            {source}  frames {result['odom_frames']}")
    print(f"matched poses       {result['matched_poses']} of {result['ground_truth_poses']} "
          f"ground-truth poses ({result['odom_messages']} estimates)")
    print(f"trajectory length   {result['trajectory_length_m']:.1f} m")
    print(f"ATE RMSE / max      {result['ape_rmse_m']:.3f} m / {result['ape_max_m']:.3f} m "
          f"(position+yaw aligned; horizontal {result['ape_horizontal_rmse_m']:.3f} m; "
          f"SE(3)-aligned {result['ape_se3_rmse_m']:.3f} m)")
    if 'rte_translation_rmse_m_per_10m' in result:
        rot = (f", {result['rpe_rotation_rmse_deg_per_10m']:.2f} deg"
               if 'rpe_rotation_rmse_deg_per_10m' in result else '')
        print(f"RTE per 10 m        {result['rte_translation_rmse_m_per_10m']:.3f} m{rot} "
              f"({result['rte_segments']} segments)")
    if 'mean' in nees:
        print(f"NEES position       mean {nees['mean']:.2f} (3 = consistent), "
              f"{nees['fraction_above_chi2_95']:.1%} above chi2 95% [{nees['status']}]")
    else:
        print(f"NEES position       {nees['status']}: {nees['reason']}")
    print(f"init latency        {a['init_latency_s']:.1f} s")
    print(f"coverage            {a['coverage']:.3f} (longest gap {a['longest_gap_s']:.2f} s, "
          f"{a['output_rate_hz']:.1f} Hz)")
    hg = result['health_gate']
    if hg['divergence_onset_s'] is not None:
        caught = (f"gate ERROR at {hg['gate_error_s']:.1f} s ({hg['detection_latency_s']:+.1f} s)"
                  if hg['caught'] else 'NOT caught by the health gate')
        print(f"divergence          at {hg['divergence_onset_s']:.1f} s; {caught}")
    elif hg['false_alarm']:
        print(f"divergence          none, but the gate fired at {hg['gate_error_s']:.1f} s (false alarm)")
    else:
        print('divergence          none (estimate speed tracks ground truth)')
    if 'healthy_ape_rmse_m' in hg:
        print(f"healthy part        ATE {hg['healthy_ape_rmse_m']:.3f} m over "
              f"{hg['healthy_trajectory_length_m']:.0f} m (before the gate fired)")
    if result.get('compute'):
        c = result['compute']
        print(f"processing / update {c['p50_ms']:.1f} ms median, {c['p95_ms']:.1f} ms p95, "
              f"{c['max_ms']:.0f} ms max (frame period {c['frame_period_ms']:.0f} ms; "
              f"{c['fraction_over_period']:.1%} over)")
    if monitored:
        worst = max(fractions.items(), key=lambda kv: kv[1], default=('none', 1.0))
        print(f"input health        worst: {worst[0]} degraded {worst[1]:.1%} of the time")
    if degraded:
        print('RESULT              INVALID: sensor inputs were degraded during replay '
              '(dropped/late messages); rerun on a less loaded machine or with --rate lower')
        return 3
    if not limits:
        print('RESULT              REPORTED (no acceptance limits given)')
        return 0
    print('RESULT              ' + ('PASS' if not failures else 'FAIL: ' + '; '.join(failures)))
    return 0 if not failures else 1


if __name__ == '__main__':
    sys.exit(main())
