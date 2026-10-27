#!/usr/bin/env bash
# Dry run of the robot entry point without the robot: a recorded /vio/* bag stands in for
# the navigation camera driver, and scripts/run_robot_vio.sh runs exactly as on the Orin
# (wall clock, no TF, recording on). Checks the safety boundary from the recording.
#
#   /ppbv2_vio/scripts/robot_dryrun.sh --calibration DIR --bag BAG [--seconds 180] [--rate 0.5]
#
# Run inside the robot container overlay (docker/compose.robot.yaml) or the laptop one.
set -uo pipefail

usage() { sed -n '2,8p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }
CAL=""; BAG=""; SECONDS_WALL=180; RATE=0.5
while [ $# -gt 0 ]; do
  case "$1" in
    --calibration) CAL=$2; shift 2 ;;
    --bag) BAG=$2; shift 2 ;;
    --seconds) SECONDS_WALL=$2; shift 2 ;;
    --rate) RATE=$2; shift 2 ;;
    *) usage ;;
  esac
done
[ -n "$CAL" ] && [ -n "$BAG" ] || usage

REPO=/ppbv2_vio
source "$REPO/scripts/vio_env.sh" openvins
RESTORE_SIGINT='import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])'
LOG=$(mktemp -d)

echo "== stand-in camera driver: $BAG at ${RATE}x for ${SECONDS_WALL} s"
timeout -s INT "$SECONDS_WALL" ros2 bag play "$BAG" --rate "$RATE" --topics \
  /vio/cam0/image_raw /vio/cam1/image_raw /vio/cam0/camera_info /vio/cam1/camera_info /vio/imu \
  > "$LOG/play.log" 2>&1 &
PLAY=$!
setsid python3 -c "$RESTORE_SIGINT" "$REPO/scripts/run_robot_vio.sh" --calibration "$CAL" \
  > "$LOG/robot.log" 2>&1 &
ROBOT=$!
wait "$PLAY"
sleep 5
kill -INT "$ROBOT" 2>/dev/null
for _ in $(seq 60); do kill -0 "$ROBOT" 2>/dev/null || break; sleep 0.5; done
head -8 "$LOG/robot.log"

RUN=$(ls -d /data/robot_runs/*_openvins_ppbv2 2>/dev/null | tail -1)
[ -n "$RUN" ] && [ -d "$RUN/launch/record" ] || { echo "FAIL: no robot run recorded"; tail -20 "$LOG/robot.log"; exit 1; }
echo "== run: $RUN"
/opt/vio-tools/bin/python -I - "$RUN/launch/record" <<'EOF'
import sys
from pathlib import Path
from rosbags.highlevel import AnyReader
with AnyReader([Path(sys.argv[1])]) as r:
    counts = {c.topic: c.msgcount for c in r.connections}
print('recorded:', counts)
problems = []
if counts.get('/vio/odom', 0) == 0:
    problems.append('no /vio/odom output')
if counts.get('/tf', 0):
    problems.append(f"/tf has {counts['/tf']} messages: the VIO must publish no TF")
for t in counts:
    if t == '/robot/odom' or t.startswith('/cmd_vel'):
        problems.append(f'forbidden topic recorded: {t}')
print('SAFETY BOUNDARY: ' + ('OK' if not problems else 'FAIL: ' + '; '.join(problems)))
sys.exit(1 if problems else 0)
EOF
