#!/usr/bin/env bash
# Build ppb_vio into /ppb_ws and run every unit test (ppb_vio + offline tools).
# Run inside the container:  /ppbv2_vio/scripts/build_ppb_vio.sh
set -euo pipefail

REPO=/ppbv2_vio
set +u; source /opt/ros/humble/setup.bash; set -u

colcon --log-base /ppb_ws/log build \
  --base-paths "$REPO/src" \
  --build-base /ppb_ws/build \
  --install-base /ppb_ws/install \
  --symlink-install \
  --event-handlers console_direct-

echo "== ppb_vio unit tests (ROS system Python)"
(cd "$REPO/src/ppb_vio" && python3 -m pytest -q -p no:cacheprovider test)

# -I: isolated mode, so the ROS PYTHONPATH cannot leak into the tools venv.
echo "== offline tool tests (/opt/vio-tools)"
(cd "$REPO/tools" && /opt/vio-tools/bin/python -I -m pytest -q -p no:cacheprovider test)
