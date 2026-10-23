# PPBv2 VIO development / replay image.
#
# ROS 2 Humble on Ubuntu 22.04 (same distro as the Jetson AGX Orin).
# Builds two estimators into SEPARATE workspaces (OpenVINS first, so that
# changing a MINS patch does not rebuild OpenVINS), because MINS vendors its own
# copy of OpenVINS' ov_core and the package names collide:
#   /opt/vio/mins_ws      MINS (primary estimator), with PPBv2 patches applied
#   /opt/vio/openvins_ws  OpenVINS (reference / debug baseline)
# Never source both workspaces in the same shell; scripts/vio_env.sh picks one.
#
# Offline evaluation tools (evo, rosbags) live in /opt/vio-tools, a virtualenv
# isolated from the ROS system Python.
#
# Build from PPBv2_VIO/:  docker compose -f docker/compose.yaml build

# ros:humble-perception-jammy, multi-arch index digest (pinned 2026-10-12).
FROM ros:humble-perception-jammy@sha256:de170d8905daf457ab2ab4f1806b8031412f16d7bef1db1cdea704e5d6626447

# Upstream estimator sources, pinned to exact commits.
ARG MINS_REPO=https://github.com/rpng/MINS.git
ARG MINS_COMMIT=e245da649c5eba5f7dfdade2df8731f2ed676590
ARG OPENVINS_REPO=https://github.com/rpng/open_vins.git
ARG OPENVINS_COMMIT=69488123ed9362dd44b6f28e7f4680abbff1442b
# Parallelism: keep low on 8 GB machines; raise on the Orin (e.g. BUILD_JOBS=6).
ARG BUILD_JOBS=2

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    PYTHONDONTWRITEBYTECODE=1

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        libboost-date-time-dev \
        libboost-filesystem-dev \
        libboost-system-dev \
        libboost-thread-dev \
        libceres-dev \
        libeigen3-dev \
        libgflags-dev \
        libgoogle-glog-dev \
        python3-colcon-common-extensions \
        python3-numpy \
        python3-pytest \
        python3-venv \
        python3-yaml \
        ros-humble-diagnostic-msgs \
        ros-humble-foxglove-bridge \
        ros-humble-rosbag2-storage-mcap \
        ros-humble-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# OpenVINS (reference baseline). ov_eval is skipped; evo is used instead.
# ---------------------------------------------------------------------------
RUN mkdir -p /opt/vio/openvins_ws/src \
    && git clone --quiet "${OPENVINS_REPO}" /opt/vio/openvins_ws/src/open_vins \
    && cd /opt/vio/openvins_ws/src/open_vins \
    && git checkout --quiet "${OPENVINS_COMMIT}" \
    && test "$(git rev-parse HEAD)" = "${OPENVINS_COMMIT}"

RUN source /opt/ros/humble/setup.bash \
    && cd /opt/vio/openvins_ws \
    && export MAKEFLAGS="-j${BUILD_JOBS}" \
    && colcon build --packages-up-to ov_msckf --parallel-workers 1 \
         --event-handlers console_direct- --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log

# ---------------------------------------------------------------------------
# MINS (camera + IMU first; wheel / GNSS enabled later through config only)
# ---------------------------------------------------------------------------
COPY patches/mins /opt/vio/patches/mins
RUN mkdir -p /opt/vio/mins_ws/src \
    && git clone --quiet "${MINS_REPO}" /opt/vio/mins_ws/src/MINS \
    && cd /opt/vio/mins_ws/src/MINS \
    && git checkout --quiet "${MINS_COMMIT}" \
    && test "$(git rev-parse HEAD)" = "${MINS_COMMIT}" \
    && for p in /opt/vio/patches/mins/*.patch; do git apply --verbose "$p"; done

# Mirrors upstream build_ros2.sh (thirdparty -> bundled OpenVINS -> mins),
# with Release flags and bounded parallelism. mins_eval is not needed.
RUN source /opt/ros/humble/setup.bash \
    && cd /opt/vio/mins_ws \
    && export MAKEFLAGS="-j${BUILD_JOBS}" \
    && COLCON_ARGS=(--parallel-workers 1 --event-handlers console_direct- --cmake-args -DCMAKE_BUILD_TYPE=Release) \
    && SRC=/opt/vio/mins_ws/src/MINS \
    && THIRDPARTY_PKGS=$(find "$SRC/thirdparty" -mindepth 1 -maxdepth 1 -type d -exec test -f '{}/package.xml' \; -print) \
    && if [ -n "$THIRDPARTY_PKGS" ]; then colcon build --paths $THIRDPARTY_PKGS "${COLCON_ARGS[@]}"; source install/setup.bash; fi \
    && colcon build --paths "$SRC"/thirdparty/open_vins/* "${COLCON_ARGS[@]}" \
    && source install/setup.bash \
    && colcon build --paths "$SRC/mins" "$SRC/mins_data" "${COLCON_ARGS[@]}" \
    && rm -rf build log

# Offline evaluation tooling, isolated from ROS' Python packages. Installed after
# the estimator builds so changing a tool pin does not rebuild MINS/OpenVINS.
COPY docker/requirements-tools.lock.txt /tmp/requirements-tools.lock.txt
RUN python3 -m venv /opt/vio-tools \
    && /opt/vio-tools/bin/python -I -m pip install --no-cache-dir --upgrade pip==24.2 \
    && /opt/vio-tools/bin/python -I -m pip install --no-cache-dir --no-deps \
         -r /tmp/requirements-tools.lock.txt \
    && /opt/vio-tools/bin/python -I -m pip check \
    && rm /tmp/requirements-tools.lock.txt

# Record exactly what was built.
RUN { echo "mins ${MINS_COMMIT}"; echo "open_vins ${OPENVINS_COMMIT}"; \
      echo "mins_patches $(cd /opt/vio/patches/mins && sha256sum *.patch | tr '\n' ' ')"; } \
    > /opt/vio/VERSIONS

# Non-root user for day-to-day work. PPBv2_VIO is bind-mounted at /ppbv2_vio.
RUN useradd --create-home --shell /bin/bash --uid 1000 vio \
    && mkdir -p /ppb_ws /data \
    && chown vio:vio /ppb_ws /data
USER vio
WORKDIR /ppbv2_vio

CMD ["bash"]
