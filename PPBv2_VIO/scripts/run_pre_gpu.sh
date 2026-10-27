#!/usr/bin/env bash
# One command for the pre-GPU VIO foundation: OpenVINS for the PPBv2 Amiga (Jetson Orin),
# proven before the hardware exists.
#
#   1  build + all unit tests
#   2  datasets (Rosario v1 field data; EuRoC sanity if present)
#   3  OpenVINS with the robot's profile (openvins_ppbv2) on the offline bench
#   4  stress suite (frame drop, jitter, blackout, IMU drop, mid-row start)
#   5  robot calibration path: a Kalibr-format calibration -> kalibr_to_openvins.py
#   6  robot entry point dry run: run_robot_vio.sh with a replayed bag as the camera
#      driver, safety boundary checked (no TF, no /robot/odom, no /cmd_vel*)
#   7  summary table, plot, compute budget
#
# Run on the host, from PPBv2_VIO/ (laptop or Orin):
#   scripts/run_pre_gpu.sh                    # build the image, then everything
#   scripts/run_pre_gpu.sh --skip-build       # reuse the built image
#   scripts/run_pre_gpu.sh --no-stress        # skip the stress suite (~10 min)
#   scripts/run_pre_gpu.sh --robot            # use docker/compose.robot.yaml (on the Orin)
#
# Datasets (git-ignored, PPBv2_VIO/data/): Rosario v1 sequence02 is fetched and converted
# automatically (~2.8 GB download). EuRoC V1_01_easy cannot be fetched by script (ETH
# rate-limits); without it the EuRoC sanity run is skipped.
set -euo pipefail

EUROC_SEQ=V1_01_easy
ROSARIO_SEQ=sequence02
SKIP_BUILD=false; STRESS=true; ROBOT=false; IN_CONTAINER=false
while [ $# -gt 0 ]; do
  case "$1" in
    --skip-build) SKIP_BUILD=true; shift ;;
    --no-stress) STRESS=false; shift ;;
    --robot) ROBOT=true; shift ;;
    --in-container) IN_CONTAINER=true; shift ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

# ---------------------------------------------------------------------------
# Host: build the image, then re-run this script inside the container.
# ---------------------------------------------------------------------------
if [ "$IN_CONTAINER" = false ]; then
  cd "$(dirname "$0")/.."
  [ -f docker/compose.yaml ] || { echo "run from PPBv2_VIO/ (docker/compose.yaml not found)"; exit 1; }
  COMPOSE=(-f docker/compose.yaml)
  [ "$ROBOT" = true ] && COMPOSE+=(-f docker/compose.robot.yaml)
  if [ "$SKIP_BUILD" = false ]; then
    echo "== building ppbv2-vio:humble (first time ~1 h on an 8 GB laptop)"
    docker compose "${COMPOSE[@]}" build
  fi
  ARGS=(--in-container)
  [ "$STRESS" = false ] && ARGS+=(--no-stress)
  exec docker compose "${COMPOSE[@]}" run --rm vio /ppbv2_vio/scripts/run_pre_gpu.sh "${ARGS[@]}"
fi

# ---------------------------------------------------------------------------
# Container.
# ---------------------------------------------------------------------------
REPO=/ppbv2_vio
PY=(/opt/vio-tools/bin/python -I)
gate() { echo; echo "######## $* ########"; }
RESULTS=()
record() { if "$@"; then RESULTS+=("PASS  $LABEL"); else RESULTS+=("FAIL  $LABEL"); fi; }

gate "1  build + all unit tests"
"$REPO/scripts/build_ppb_vio.sh"

gate "2  datasets"
R_N=$(echo "$ROSARIO_SEQ" | grep -oE '[0-9]+$' | sed 's/^0*//')
R_DIR=/data/rosario_v1/$ROSARIO_SEQ
if [ ! -d "$R_DIR/ros2_bag" ]; then
  "$REPO/scripts/fetch_rosario_v1.sh" "$R_N" "$R_DIR/download"
  "${PY[@]}" "$REPO/tools/prepare_rosario_v1.py" "$R_N" "$R_DIR/download" "$R_DIR" --delete-parts
else
  "${PY[@]}" "$REPO/tools/prepare_rosario_v1.py" "$R_N" "$R_DIR/download" "$R_DIR" --configs-only > /dev/null
  echo "-- Rosario $ROSARIO_SEQ ready"
fi
E_DIR=/data/euroc/$EUROC_SEQ
EUROC=true
EUROC_GT=/opt/vio/openvins_ws/src/open_vins/ov_data/euroc_mav/$EUROC_SEQ.txt
if [ ! -d "$E_DIR/ros2_bag" ]; then
  if [ -d "$E_DIR/$EUROC_SEQ/mav0" ]; then
    "${PY[@]}" "$REPO/tools/prepare_asl.py" "$E_DIR/$EUROC_SEQ" "$E_DIR/ros2_bag" --ground-truth "$EUROC_GT"
  else
    EUROC=false
    echo "-- EuRoC $EUROC_SEQ not found at $E_DIR/$EUROC_SEQ/mav0 (browser download, see README); sanity run skipped"
  fi
elif [ ! -f "$E_DIR/dataset.yaml" ]; then
  "${PY[@]}" "$REPO/tools/prepare_asl.py" "$E_DIR/$EUROC_SEQ" "$E_DIR/ros2_bag" \
    --descriptor-only --sequence "$EUROC_SEQ" --ground-truth "$EUROC_GT"
fi

gate "3  OpenVINS, robot profile (openvins_ppbv2), offline bench"
if [ "$EUROC" = true ]; then
  LABEL="EuRoC $EUROC_SEQ sanity (openvins_euroc)"
  record bash -o pipefail -c "'$REPO/scripts/run_bench.sh' openvins euroc '$EUROC_SEQ' --tag pre_gpu | tail -12"
fi
LABEL="Rosario $ROSARIO_SEQ, openvins_ppbv2 (VIO alone < 2 m ATE)"
record bash -o pipefail -c "'$REPO/scripts/run_bench.sh' openvins rosario_v1 '$ROSARIO_SEQ' --profile openvins_ppbv2 --tag pre_gpu | tail -14"

if [ "$STRESS" = true ]; then
  gate "4  stress suite, openvins_ppbv2"
  "$REPO/scripts/run_stress_suite.sh" openvins rosario_v1 "$ROSARIO_SEQ" --profile openvins_ppbv2
fi

gate "5  robot calibration path (Kalibr format -> kalibr_to_openvins.py)"
CAL=/data/calibration/pre_gpu_check
rm -rf "$CAL" "$CAL.inputs"; mkdir -p "$CAL.inputs"
cp "$R_DIR/download/calibration$(printf '%02d' "$R_N").yaml" "$CAL.inputs/camchain-imucam.yaml"
printf 'accelerometer_noise_density: 0.1\naccelerometer_random_walk: 0.001\ngyroscope_noise_density: 0.01\ngyroscope_random_walk: 0.0001\nupdate_rate: 142.0\n' \
  > "$CAL.inputs/imu.yaml"
LABEL="kalibr_to_openvins (Rosario calibration in Kalibr format)"
record "${PY[@]}" "$REPO/tools/kalibr_to_openvins.py" --camchain "$CAL.inputs/camchain-imucam.yaml" \
  --imu "$CAL.inputs/imu.yaml" --imu-in-base-xyz 1.96 -0.017 1.02 \
  --imu-in-base-quat 0.81281825 -0.00799763 0.58239402 0.00892938 --camera-hz 15 --out "$CAL"

gate "6  robot entry point dry run (run_robot_vio.sh, replayed bag as the camera driver)"
LABEL="run_robot_vio.sh dry run + safety boundary"
record "$REPO/scripts/robot_dryrun.sh" --calibration "$CAL" --bag "$R_DIR/ros2_bag" --seconds 120

gate "7  summary"
"${PY[@]}" "$REPO/tools/bench_summary.py" /data/runs/*_bench_* --out /data/bench/summary > /dev/null
latest() { ls -d /data/runs/*_bench_openvins_"$1"_pre_gpu 2>/dev/null | tail -1; }
PANELS=()
if [ "$EUROC" = true ] && [ -n "$(latest "$EUROC_SEQ")" ]; then
  PANELS+=(--panel "$E_DIR/dataset.yaml" "OpenVINS=$(latest "$EUROC_SEQ")/estimate.tum")
fi
PANELS+=(--panel "$R_DIR/dataset.yaml" "OpenVINS, openvins_ppbv2=$(latest "$ROSARIO_SEQ")/estimate.tum")
"${PY[@]}" "$REPO/tools/plot_trajectories.py" --out /data/plots/pre_gpu.png \
  --title "PPBv2 VIO foundation, pre-GPU check (OpenVINS, offline bench)" "${PANELS[@]}" > /dev/null
grep -h "processing / update" "$(latest "$ROSARIO_SEQ")/summary.txt" || true
echo
printf '%s\n' "${RESULTS[@]}"
echo
echo "table: data/bench/summary.md   plot: data/plots/pre_gpu.png   runs: data/runs/, data/robot_runs/"
