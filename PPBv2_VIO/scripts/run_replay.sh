#!/usr/bin/env bash
# Replay one converted dataset sequence through an estimator and score /vio/odom.
#
#   /ppbv2_vio/scripts/run_replay.sh <mins|openvins> <euroc|tumvi|rosario_v1> <SEQUENCE> [--rate R] [--foxglove]
#
# Expects /data/<dataset>/<SEQUENCE>/ros2_bag (tools/prepare_asl.py, prepare_rosario_v1.py).
# Uses that folder's dataset.yaml, platform.yaml and <estimator>/ calibration when present.
# Writes /data/runs/<UTC time>_<estimator>_<SEQUENCE>/ and exits non-zero if the
# run fails or misses the acceptance limits in tools/acceptance.yaml.
set -euo pipefail

usage() { sed -n '2,8p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }
[ $# -ge 3 ] || usage
ESTIMATOR=$1; DATASET=$2; SEQUENCE=$3; shift 3
RATE=1.0; FOXGLOVE=false
while [ $# -gt 0 ]; do
  case "$1" in
    --rate) RATE=$2; shift 2 ;;
    --foxglove) FOXGLOVE=true; shift ;;
    *) usage ;;
  esac
done

REPO=/ppbv2_vio
DS_DIR=/data/$DATASET/$SEQUENCE
BAG=$DS_DIR/ros2_bag
RUN_DIR=/data/runs/$(date -u +%Y%m%dT%H%M%SZ)_${ESTIMATOR}_${SEQUENCE}
[ -d "$BAG" ] || { echo "missing $BAG (run the dataset loader first)" >&2; exit 1; }
# Ground truth: the loader's dataset.yaml, else OpenVINS' ov_data (pinned with the
# OpenVINS commit; its V1_01_easy orientation is re-estimated and more accurate).
if [ -f "$DS_DIR/dataset.yaml" ]; then
  EVAL_ARGS=(--dataset "$DS_DIR/dataset.yaml")
else
  case "$DATASET" in
    euroc) GT_DIR=euroc_mav ;;
    tumvi) GT_DIR=tum_vi ;;
    *) echo "no dataset.yaml in $DS_DIR" >&2; exit 1 ;;
  esac
  GT=/opt/vio/openvins_ws/src/open_vins/ov_data/$GT_DIR/$SEQUENCE.txt
  [ -f "$GT" ] || { echo "no ground truth for $SEQUENCE at $GT" >&2; exit 1; }
  EVAL_ARGS=(--gt "$GT" --sensor-bag "$BAG" --sequence "$SEQUENCE")
fi
# Per-sequence platform (base_link -> imu mounting) and estimator calibration, if generated.
PLATFORM=$DATASET
[ -f "$DS_DIR/platform.yaml" ] && PLATFORM=$DS_DIR/platform.yaml
CALIB_DIR=""
[ -d "$DS_DIR/$ESTIMATOR" ] && CALIB_DIR=$DS_DIR/$ESTIMATOR

source "$REPO/scripts/vio_env.sh" "$ESTIMATOR"
mkdir -p "$RUN_DIR"
{
  echo "estimator: $ESTIMATOR"
  echo "dataset: $DATASET"
  echo "sequence: $SEQUENCE"
  echo "playback_rate: $RATE"
  echo "repo_commit: $(git -c safe.directory='*' -C "$REPO" rev-parse HEAD 2>/dev/null || echo unknown)"
  echo "repo_dirty: $(git -c safe.directory='*' -C "$REPO" status --porcelain 2>/dev/null | grep -q . && echo true || echo false)"
  sed 's/^/upstream: /' /opt/vio/VERSIONS
} > "$RUN_DIR/run_info.yaml"
# The launch file refuses a non-empty run_dir, so it writes into a subfolder.
LAUNCH_DIR=$RUN_DIR/launch
LOG=$RUN_DIR/launch.log

echo "== run directory: $RUN_DIR"
# Background jobs of a non-interactive shell start with SIGINT ignored, and
# `ros2 launch` keeps that disposition, so it could not be stopped cleanly.
# Restore the default before exec'ing launch.
RESTORE_SIGINT='import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])'
setsid python3 -c "$RESTORE_SIGINT" ros2 launch ppb_vio vio.launch.py \
  estimator:="${ESTIMATOR}_${DATASET}" platform:="$PLATFORM" run_dir:="$LAUNCH_DIR" \
  calibration_dir:="$CALIB_DIR" \
  use_sim_time:=true publish_tf:=true foxglove:="$FOXGLOVE" > "$LOG" 2>&1 &
LAUNCH_PID=$!
UNCLEAN_STOP=0

# SIGINT goes to `ros2 launch` only; it shuts its children down in order. The
# whole process group is terminated only if that has not finished in 30 s.
stop_launch() {
  if kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    for _ in $(seq 60); do kill -0 "$LAUNCH_PID" 2>/dev/null || break; sleep 0.5; done
    if kill -0 "$LAUNCH_PID" 2>/dev/null; then
      UNCLEAN_STOP=1
      echo "launch did not stop after SIGINT; terminating process group" >&2
      kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
    fi
  fi
}
trap stop_launch EXIT

# Guard against MINS subscribing to sensor_msgs/CompressedImage (upstream reads
# an uninitialized flag; fixed by patches/mins/0002, checked here regardless).
check_raw_image_subscription() {
  if grep -q "subscribing to cam (.*compressed)" "$LOG"; then
    echo "estimator subscribed to compressed images, but /vio/* carries raw images; see $LOG" >&2
    exit 1
  fi
}

# Ready = every input topic has both of its subscribers (estimator + stream
# monitor) and the recorder has subscribed to /vio/odom. Starting the replay
# earlier would silently drop the first seconds the estimator initializes from.
subscribers() { ros2 topic info "$1" 2>/dev/null | sed -n 's/^Subscription count: //p'; }
echo "== waiting for pipeline to come up"
for i in $(seq 120); do
  kill -0 "$LAUNCH_PID" 2>/dev/null || { echo "launch exited early; see $LOG" >&2; tail -20 "$LOG" >&2; exit 1; }
  ready=1
  for t in /vio/imu /vio/cam0/image_raw /vio/cam1/image_raw; do
    [ "$(subscribers "$t")" -ge 2 ] 2>/dev/null || ready=0
  done
  [ "$(subscribers /vio/odom)" -ge 1 ] 2>/dev/null || ready=0
  [ "$ready" = 1 ] && break
  [ "$i" -eq 120 ] && { echo "pipeline not ready after 60 s; see $LOG" >&2; exit 1; }
  sleep 0.5
done

check_raw_image_subscription
echo "== replaying $SEQUENCE at ${RATE}x"
ros2 bag play "$BAG" --clock 200 --rate "$RATE" > "$RUN_DIR/play.log" 2>&1 \
  || { echo "ros2 bag play failed; see $RUN_DIR/play.log" >&2; exit 1; }
kill -0 "$LAUNCH_PID" 2>/dev/null || { echo "pipeline died during replay; see $LOG" >&2; exit 1; }
sleep 3   # let the estimator drain its queue and the recorder flush
stop_launch
trap - EXIT
check_raw_image_subscription
# Any process dying before shutdown starts is fatal, as is any of our own nodes
# or the recorder dying at any time. An estimator crash *after* SIGINT is only a
# warning: OpenVINS segfaults on shutdown under Humble, and by then every output
# has been recorded.
SHUTDOWN_LINE=$(grep -n -m1 "sending signal 'SIGINT'" "$LOG" | cut -d: -f1)
FATAL=0
while IFS=: read -r line text; do
  if [ -n "$SHUTDOWN_LINE" ] && [ "$line" -gt "$SHUTDOWN_LINE" ] \
     && echo "$text" | grep -Eq '^\[ERROR\] \[(subscribe|run_subscribe_msckf)-[0-9]+\]'; then
    echo "warning: estimator crashed during shutdown (outputs already recorded): $text" >&2
  else
    echo "a process crashed or exited with an error: $text" >&2
    FATAL=1
  fi
done < <(grep -n "process has died" "$LOG")
[ "$FATAL" = 0 ] || { echo "see $LOG" >&2; exit 1; }

ln -s launch/record "$RUN_DIR/record"
echo "== evaluating"
/opt/vio-tools/bin/python -I "$REPO/tools/evaluate_run.py" \
  --run-dir "$RUN_DIR" "${EVAL_ARGS[@]}" \
  --acceptance "$REPO/tools/acceptance.yaml" | tee "$RUN_DIR/summary.txt"
if [ "$UNCLEAN_STOP" = 1 ]; then
  echo "FAIL: the pipeline had to be force-terminated; results above are not trusted" >&2
  exit 1
fi
