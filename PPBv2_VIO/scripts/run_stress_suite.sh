#!/usr/bin/env bash
# VIO stress suite (spec: "VIO frontend selection" — robustness under induced faults).
# For each case: write a stressed copy of the sequence (tools/stress_bag.py, seeded),
# run the offline bench on it, keep the stress record in the run folder, delete the
# stressed bag (~2 GB each; only one exists at a time).
#
#   /ppbv2_vio/scripts/run_stress_suite.sh <openvins> <rosario_v1|euroc> <SEQUENCE> [--profile P] [--only CASE]
#
# Cases (seed 0):
#   frame_drop_10   10 % of stereo pairs dropped
#   jitter_5ms      camera stamps jittered by N(0, 5 ms)
#   blackout_2s     no camera frames for 2 s starting 60 s in (tracking loss)
#   imu_drop_2      2 % of IMU samples dropped
#   start_60s       initialize 60 s in (at speed, mid-row)
set -euo pipefail

usage() { sed -n '2,15p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }
[ $# -ge 3 ] || usage
FRONTEND=$1; DATASET=$2; SEQUENCE=$3; shift 3
PROFILE_ARGS=(); ONLY=""
while [ $# -gt 0 ]; do
  case "$1" in
    --profile) PROFILE_ARGS=(--profile "$2"); shift 2 ;;
    --only) ONLY=$2; shift 2 ;;
    *) usage ;;
  esac
done

REPO=/ppbv2_vio
BAG=/data/$DATASET/$SEQUENCE/ros2_bag
STRESS_DIR=/data/stress
mkdir -p "$STRESS_DIR"
declare -A CASES=(
  [frame_drop_10]="--drop-frames 0.10"
  [jitter_5ms]="--jitter-ms 5"
  [blackout_2s]="--blackout 60:62"
  [imu_drop_2]="--imu-drop 0.02"
  [start_60s]="--start 60"
)
ORDER=(frame_drop_10 jitter_5ms blackout_2s imu_drop_2 start_60s)

for name in "${ORDER[@]}"; do
  [ -n "$ONLY" ] && [ "$ONLY" != "$name" ] && continue
  out=$STRESS_DIR/${SEQUENCE}_$name
  rm -rf "$out" "$out.stress.json"
  echo "######## stress: $name (${CASES[$name]})"
  # shellcheck disable=SC2086
  /opt/vio-tools/bin/python -I "$REPO/tools/stress_bag.py" "$BAG" "$out" ${CASES[$name]} --seed 0 \
    > /dev/null
  set +e
  "$REPO/scripts/run_bench.sh" "$FRONTEND" "$DATASET" "$SEQUENCE" --bag "$out" \
    --tag "stress_$name" "${PROFILE_ARGS[@]}" | grep -E "ATE RMSE|divergence|healthy|init latency|coverage|RESULT"
  set -e
  run=$(ls -d /data/runs/*_bench_${FRONTEND}_${SEQUENCE}_stress_$name | tail -1)
  cp "$out.stress.json" "$run/stress.json"
  rm -rf "$out" "$out.stress.json"
done
