#!/usr/bin/env bash
# Local fusion run (spec Task 3): replay fusion inputs through vio_gate + robot_localization
# ekf_local (the nodes that run on the robot), record /odometry/local, score it.
#
#   /ppbv2_vio/scripts/run_fusion.sh <rosario_v1|ppbv2> <SEQUENCE> --vio-run RUN_DIR \
#       [--variants v1,v2] [--vio-blackout T0:T1] [--tag TAG] [--rate R]
#
#   --vio-run       an offline bench run of the same sequence (its record/ has /vio/odom, /vio/status)
#   --variants      config/variants/*.yaml layered on ekf_local.yaml (e.g. vio_twist,imu_gyro_only)
#   --vio-blackout  remove VIO output in a window (s after the recording start): VIO lost
#
# Inputs are built once per (sequence, VIO run, blackout) by tools/prepare_fusion_inputs.py
# into /data/fusion/ (no images: small, replays live without drops).
# Writes /data/runs/<UTC>_fusion_local_<SEQUENCE>[_TAG]/: record/, evaluation.json, summary.txt.
set -euo pipefail

usage() { sed -n '2,14p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }
[ $# -ge 2 ] || usage
DATASET=$1; SEQUENCE=$2; shift 2
VIO_RUN=""; VARIANTS=""; BLACKOUT=""; TAG=""; RATE=1.0
while [ $# -gt 0 ]; do
  case "$1" in
    --vio-run) VIO_RUN=$2; shift 2 ;;
    --variants) VARIANTS=$2; shift 2 ;;
    --vio-blackout) BLACKOUT=$2; shift 2 ;;
    --tag) TAG=$2; shift 2 ;;
    --rate) RATE=$2; shift 2 ;;
    *) usage ;;
  esac
done
[ -n "$VIO_RUN" ] && [ -d "$VIO_RUN/record" ] || { echo "need --vio-run with a record/ folder" >&2; exit 2; }

REPO=/ppbv2_vio
PY=(/opt/vio-tools/bin/python -I)
DS_DIR=/data/$DATASET/$SEQUENCE
WHEEL_FRAME=base_link
[ "$DATASET" = rosario_v1 ] && WHEEL_FRAME=odom       # Rosario v1 records wheel twist in odom
INPUTS=/data/fusion/${SEQUENCE}__$(basename "$VIO_RUN")${BLACKOUT:+__vio_blackout_${BLACKOUT/:/_}}
if [ ! -d "$INPUTS" ]; then
  mkdir -p /data/fusion
  BL_ARGS=(); [ -n "$BLACKOUT" ] && BL_ARGS=(--vio-blackout "$BLACKOUT")
  "${PY[@]}" "$REPO/tools/prepare_fusion_inputs.py" --recording "$DS_DIR/ros2_bag" \
    --vio-run "$VIO_RUN" --out "$INPUTS" --wheel-twist-frame "$WHEEL_FRAME" "${BL_ARGS[@]}" > /dev/null
fi

RUN_DIR=/data/runs/$(date -u +%Y%m%dT%H%M%SZ)_fusion_local_${SEQUENCE}${TAG:+_$TAG}
mkdir -p "$RUN_DIR"
source "$REPO/scripts/vio_env.sh" openvins
{
  echo "mode: fusion_local"
  echo "dataset: $DATASET"
  echo "sequence: $SEQUENCE"
  echo "frontend: fusion_local"
  echo "profile: ekf_local${VARIANTS:++$VARIANTS}"
  echo "inputs: $INPUTS"
  echo "vio_run: $VIO_RUN"
  echo "vio_blackout_s: ${BLACKOUT:-none}"
  echo "input_bag: $INPUTS"
  echo "repo_commit: $(git -c safe.directory='*' -C "$REPO" rev-parse HEAD 2>/dev/null || echo unknown)"
  sed 's/^/upstream: /' /opt/vio/VERSIONS
} > "$RUN_DIR/run_info.yaml"
echo "== run directory: $RUN_DIR"

RESTORE_SIGINT='import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])'
# ros2 launch rejects an empty "name:=" argument, so pass variants only when given.
VAR_ARGS=(); [ -n "$VARIANTS" ] && VAR_ARGS=(variants:="$VARIANTS")
setsid python3 -c "$RESTORE_SIGINT" ros2 launch ppbv2_localization local_ekf.launch.py \
  use_sim_time:=true "${VAR_ARGS[@]}" > "$RUN_DIR/launch.log" 2>&1 &
LAUNCH=$!
setsid python3 -c "$RESTORE_SIGINT" ros2 bag record --storage mcap -o "$RUN_DIR/record" \
  /odometry/local /localization/vio_gate /diagnostics > "$RUN_DIR/record.log" 2>&1 &
RECORD=$!
stop() {
  for pid in $RECORD $LAUNCH; do kill -INT "$pid" 2>/dev/null || true; done
  for _ in $(seq 40); do kill -0 $LAUNCH 2>/dev/null || kill -0 $RECORD 2>/dev/null || break; sleep 0.5; done
}
trap stop EXIT

subs() { ros2 topic info "$1" 2>/dev/null | sed -n 's/^Subscription count: //p'; }
for i in $(seq 60); do
  ready=1
  for t in /imu/data /wheel/odom /vio/odom /vio/status /odometry/local; do
    [ "$(subs "$t")" -ge 1 ] 2>/dev/null || ready=0
  done
  [ "$ready" = 1 ] && break
  [ "$i" -eq 60 ] && { echo "fusion nodes not ready after 30 s; see $RUN_DIR/launch.log" >&2; exit 1; }
  sleep 0.5
done

echo "== replaying fusion inputs at ${RATE}x"
ros2 bag play "$INPUTS" --clock 200 --rate "$RATE" > "$RUN_DIR/play.log" 2>&1
sleep 2
stop
trap - EXIT
grep -h "VIO dropped\|VIO forwarded" "$RUN_DIR/launch.log" | sed 's/^.*\] //' | head -5 || true
# A fusion node dying before shutdown invalidates the run.
SHUTDOWN_LINE=$(grep -n -m1 "sending signal 'SIGINT'" "$RUN_DIR/launch.log" | cut -d: -f1)
if grep -n "process has died" "$RUN_DIR/launch.log" | \
   awk -F: -v s="${SHUTDOWN_LINE:-999999999}" '$1 < s' | grep -q .; then
  echo "FAIL: a fusion node died during the run; see $RUN_DIR/launch.log" | tee "$RUN_DIR/summary.txt" >&2
  exit 1
fi

echo "== evaluating /odometry/local"
"${PY[@]}" "$REPO/tools/evaluate_run.py" --run-dir "$RUN_DIR" --estimate-bag "$RUN_DIR/record" \
  --topic /odometry/local --dataset "$DS_DIR/dataset.yaml" | tee "$RUN_DIR/summary.txt"
