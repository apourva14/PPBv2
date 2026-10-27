#!/usr/bin/env bash
# Offline (serial) VIO bench run: the frontend reads the bag directly, so no
# message can be dropped and a run is repeatable on any machine (8 GB laptop
# included). Use run_replay.sh for live end-to-end checks of the ROS nodes.
#
#   /ppbv2_vio/scripts/run_bench.sh <openvins> <euroc|rosario_v1> <SEQUENCE> \
#       [--profile NAME|PATH] [--bag PATH] [--tag TAG] [--acceptance FILE|none]
#
#   --profile  estimator profile (default <frontend>_<dataset>), e.g. a tuning variant
#   --bag      input bag instead of /data/<dataset>/<SEQUENCE>/ros2_bag (stress bags)
#   --tag      suffix for the run folder name
#
# Writes /data/runs/<UTC time>_bench_<frontend>_<SEQUENCE>[_TAG]/ with the exact
# estimator config, estimator output + timing, /vio/odom + /vio/status (record/),
# estimate.tum, evaluation.json, summary.txt. Exit: 0 pass, 1 fail, 2 usage.
set -euo pipefail

usage() { sed -n '2,15p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }
[ $# -ge 3 ] || usage
FRONTEND=$1; DATASET=$2; SEQUENCE=$3; shift 3
PROFILE=""; BAG=""; TAG=""; ACCEPTANCE=/ppbv2_vio/tools/acceptance.yaml
while [ $# -gt 0 ]; do
  case "$1" in
    --profile) PROFILE=$2; shift 2 ;;
    --bag) BAG=$2; shift 2 ;;
    --tag) TAG=$2; shift 2 ;;
    --acceptance) ACCEPTANCE=$2; shift 2 ;;
    *) usage ;;
  esac
done

REPO=/ppbv2_vio
TOOLS_PY=(/opt/vio-tools/bin/python -I)
DS_DIR=/data/$DATASET/$SEQUENCE
BAG=${BAG:-$DS_DIR/ros2_bag}
PROFILE=${PROFILE:-${FRONTEND}_${DATASET}}
[ -d "$BAG" ] || { echo "missing $BAG (run the dataset loader first)" >&2; exit 1; }
[ -f "$DS_DIR/dataset.yaml" ] || { echo "missing $DS_DIR/dataset.yaml (dataset loader)" >&2; exit 1; }
PLATFORM=$DS_DIR/platform.yaml
[ -f "$PLATFORM" ] || PLATFORM=$REPO/src/ppb_vio/config/platforms/$DATASET.yaml
CALIB_ARGS=()
[ -d "$DS_DIR/$FRONTEND" ] && CALIB_ARGS=(--calibration-dir "$DS_DIR/$FRONTEND")

RUN_DIR=/data/runs/$(date -u +%Y%m%dT%H%M%SZ)_bench_${FRONTEND}_${SEQUENCE}${TAG:+_$TAG}
mkdir -p "$RUN_DIR/estimator"
source "$REPO/scripts/vio_env.sh" "$FRONTEND"
{
  echo "mode: offline_bench"
  echo "frontend: $FRONTEND"
  echo "dataset: $DATASET"
  echo "sequence: $SEQUENCE"
  echo "profile: $PROFILE"
  echo "input_bag: $BAG"
  echo "platform: $PLATFORM"
  echo "repo_commit: $(git -c safe.directory='*' -C "$REPO" rev-parse HEAD 2>/dev/null || echo unknown)"
  echo "repo_dirty: $(git -c safe.directory='*' -C "$REPO" status --porcelain 2>/dev/null | grep -q . && echo true || echo false)"
  sed 's/^/upstream: /' /opt/vio/VERSIONS
} > "$RUN_DIR/run_info.yaml"
echo "== run directory: $RUN_DIR"

ENTRY=$(ros2 run ppb_vio prepare_estimator_config --profile "$PROFILE" --run-dir "$RUN_DIR" "${CALIB_ARGS[@]}")

echo "== $FRONTEND (serial) on $(basename "$(dirname "$BAG")")/$(basename "$BAG")"
case "$FRONTEND" in
  openvins)
    set +e
    ros2 run ppb_vio_openvins openvins_serial "$ENTRY" "$BAG" "$RUN_DIR/estimator" \
      > "$RUN_DIR/estimator.log" 2>&1
    rc=$?
    set -e
    tail -1 "$RUN_DIR/estimator.log"
    if [ "$rc" -eq 3 ]; then echo "FAIL: $FRONTEND never initialized" | tee "$RUN_DIR/summary.txt"; exit 1; fi
    [ "$rc" -eq 0 ] || { echo "$FRONTEND failed (exit $rc); see $RUN_DIR/estimator.log" >&2; exit 1; }
    ;;
  *) echo "no offline runner for frontend '$FRONTEND' yet" >&2; exit 2 ;;
esac

echo "== /vio/odom + /vio/status (offline adapter)"
"${TOOLS_PY[@]}" "$REPO/tools/offline_adapter.py" --input "$RUN_DIR/estimator/odom" \
  --profile "$RUN_DIR/profile.yaml" --platform "$PLATFORM" --output "$RUN_DIR/record" \
  > "$RUN_DIR/adapter.json"

echo "== evaluating"
ACC_ARGS=()
[ "$ACCEPTANCE" != none ] && ACC_ARGS=(--acceptance "$ACCEPTANCE")
"${TOOLS_PY[@]}" "$REPO/tools/evaluate_run.py" --run-dir "$RUN_DIR" --offline \
  --dataset "$DS_DIR/dataset.yaml" --sensor-bag "$BAG" "${ACC_ARGS[@]}" | tee "$RUN_DIR/summary.txt"
