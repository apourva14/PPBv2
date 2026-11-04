#!/usr/bin/env bash
# Build ppb_vio, ppbv2_localization (+ the OpenVINS bench runner) into /ppb_ws and run
# every unit test.
# Run inside the container:  /ppbv2_vio/scripts/build_ppb_vio.sh
set -euo pipefail

REPO=/ppbv2_vio
JOBS=${BUILD_JOBS:-2}
set +u; source /opt/ros/humble/setup.bash; set -u
# --symlink-install leaves dangling links when a source file (e.g. a config) is deleted,
# and colcon then fails copying them; drop them first.
find /ppb_ws/build /ppb_ws/install -xtype l -delete 2>/dev/null || true

colcon --log-base /ppb_ws/log build \
  --base-paths "$REPO/src" --packages-select ppb_vio ppbv2_localization \
  --build-base /ppb_ws/build \
  --install-base /ppb_ws/install \
  --symlink-install \
  --event-handlers console_direct-

# The offline OpenVINS runner links against the OpenVINS workspace, so it gets
# its own build/install tree (sourced by vio_env.sh openvins). -j bounded for 8 GB.
(
  set +u; source /opt/vio/openvins_ws/install/setup.bash; set -u
  MAKEFLAGS="-j$JOBS" colcon --log-base /ppb_ws/log_openvins build \
    --base-paths "$REPO/src" --packages-select ppb_vio_openvins \
    --build-base /ppb_ws/build_openvins \
    --install-base /ppb_ws/install_openvins \
    --parallel-workers 1 \
    --event-handlers console_direct- \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
)

echo "== ppb_vio unit tests (ROS system Python)"
(cd "$REPO/src/ppb_vio" && python3 -m pytest -q -p no:cacheprovider test)
echo "== ppbv2_localization unit tests"
(cd "$REPO/src/ppbv2_localization" && python3 -m pytest -q -p no:cacheprovider test)

# -I: isolated mode, so the ROS PYTHONPATH cannot leak into the tools venv.
echo "== offline tool tests (/opt/vio-tools)"
(cd "$REPO/tools" && /opt/vio-tools/bin/python -I -m pytest -q -p no:cacheprovider test)
