#!/usr/bin/env bash
# Run OpenVINS live on the robot (Jetson AGX Orin) on the navigation camera's /vio/* topics.
#
#   /ppbv2_vio/scripts/run_robot_vio.sh --calibration /data/calibration/<folder> [--no-record] [--foxglove]
#
# Inside the robot container (docker/compose.robot.yaml). The calibration folder comes
# from tools/kalibr_to_openvins.py. Publishes /vio/odom and /vio/status only (never
# /robot/odom or /cmd_vel*; the adapter refuses), no TF, localhost-only DDS: nothing
# reaches the navigation stack or the motors (spec P1: earn trust in stages).
# Writes /data/robot_runs/<UTC>_openvins_ppbv2/ (config used, record/ bag unless --no-record).
set -euo pipefail

usage() { sed -n '2,11p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }
CAL=""; RECORD=true; FOXGLOVE=false
while [ $# -gt 0 ]; do
  case "$1" in
    --calibration) CAL=$2; shift 2 ;;
    --no-record) RECORD=false; shift ;;
    --foxglove) FOXGLOVE=true; shift ;;
    *) usage ;;
  esac
done
[ -n "$CAL" ] || usage

REPO=/ppbv2_vio
for f in openvins/kalibr_imucam_chain.yaml openvins/kalibr_imu_chain.yaml platform.yaml \
         calibration_report.json; do
  [ -f "$CAL/$f" ] || { echo "calibration incomplete: missing $CAL/$f (run tools/kalibr_to_openvins.py)" >&2; exit 1; }
done
/opt/vio-tools/bin/python -I -c "
import json, sys
r = json.load(open('$CAL/calibration_report.json'))
for w in r['warnings']: print('calibration warning:', w, file=sys.stderr)
print(f\"calibration: baseline {r['stereo_baseline_mm']:.1f} mm, camera {r['camera_hz']:g} Hz, IMU {r['imu_hz']:g} Hz\")"

source "$REPO/scripts/vio_env.sh" openvins

# The navigation camera driver must already be publishing the /vio/* contract.
publishers() { ros2 topic info "$1" 2>/dev/null | sed -n 's/^Publisher count: //p'; }
echo "== waiting for the navigation camera driver (/vio/imu, /vio/cam0, /vio/cam1)"
for i in $(seq 60); do
  ok=1
  for t in /vio/imu /vio/cam0/image_raw /vio/cam1/image_raw; do
    [ "$(publishers "$t")" -ge 1 ] 2>/dev/null || ok=0
  done
  [ "$ok" = 1 ] && break
  [ "$i" -eq 60 ] && { echo "no publishers on /vio/* after 30 s: start the camera driver (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)" >&2; exit 1; }
  sleep 0.5
done

RUN_DIR=/data/robot_runs/$(date -u +%Y%m%dT%H%M%SZ)_openvins_ppbv2
mkdir -p "$RUN_DIR"
{
  echo "mode: robot_live"
  echo "calibration: $CAL"
  echo "calibration_report_sha256: $(sha256sum "$CAL/calibration_report.json" | cut -d' ' -f1)"
  echo "ros_domain_id: $ROS_DOMAIN_ID"
  echo "repo_commit: $(git -c safe.directory='*' -C "$REPO" rev-parse HEAD 2>/dev/null || echo unknown)"
  echo "repo_dirty: $(git -c safe.directory='*' -C "$REPO" status --porcelain 2>/dev/null | grep -q . && echo true || echo false)"
  sed 's/^/upstream: /' /opt/vio/VERSIONS
} > "$RUN_DIR/run_info.yaml"
echo "== run directory: $RUN_DIR (Ctrl-C to stop)"
exec ros2 launch ppb_vio vio.launch.py estimator:=openvins_ppbv2 platform:="$CAL/platform.yaml" \
  calibration_dir:="$CAL/openvins" run_dir:="$RUN_DIR/launch" use_sim_time:=false \
  publish_tf:=false record:="$RECORD" foxglove:="$FOXGLOVE"
