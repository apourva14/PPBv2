# PPBv2 VIO

Visual-inertial odometry (VIO) for PPBv2, developed **alongside** the existing
RTK navigation stack, not inside it. Phase 1 goal: prove that a stereo camera +
IMU estimator produces a reliable `/vio/odom` before it is fused with anything
or allowed near the motors.

- **Primary estimator:** [MINS](https://github.com/rpng/MINS), camera + IMU only
  for now (wheel and GNSS are enabled later through config).
- **Reference baseline:** [OpenVINS](https://github.com/rpng/open_vins).
- **Target computer:** Jetson AGX Orin, ROS 2 Humble. Development runs in the
  same environment in Docker (arm64 Ubuntu 22.04).

## Status at a glance (2026-10-14)

| Item | State |
|---|---|
| Docker environment (Humble, pinned MINS + OpenVINS) | done, builds reproducibly |
| `ppb_vio` package (`/vio/odom` adapter, input stream monitor, launch) | done, 49 unit tests |
| Offline tools (dataset conversion, scoring, demo file) | done, 19 unit tests |
| Pipeline verified against ground truth | **yes**, EuRoC and TUM-VI |
| MINS on EuRoC `V1_01_easy` (its maintained config) | **PASS**, 5.7 / 6.3 cm APE |
| OpenVINS on EuRoC `V1_01_easy` | **PASS**, 3.9 cm APE |
| OpenVINS on TUM-VI `room1` (fisheye) | **PASS**, 7.1–7.8 cm APE over 4 runs |
| MINS on TUM-VI `room1` (its upstream fisheye config) | **FAIL**, diverges |
| Navigation camera, IMU, calibration, robot recordings | not started (hardware) |
| Fusion with RTK / wheel odometry, control | out of scope for Phase 1 |

All work is on branch `setup-vio` and is **not committed yet**. Nothing in
`PPBv2_Navigation/` or `PPBv2_Imaging/` was changed.

## Decisions this work is based on

- New **dedicated stereo navigation camera**, global shutter, with an onboard
  hardware-synchronized IMU. The FLIR phenotyping cameras are not touched. The
  HWT905 IMU is a fallback only (its old driver in `main` history, removed in
  `50ba459`, published orientation only with host-clock stamps, so it is not
  usable for VIO as it was).
- VIO runs on the **Orin**. Camera streams never leave it; only `/vio/odom` and
  `/vio/status` will be sent to the Pi. The imaging launcher's
  `ROS_LOCALHOST_ONLY=1` / domain 77 setup stays as it is.
- **MINS** is the project estimator; **OpenVINS** is the reference used to tell
  estimator problems from pipeline problems.
- VIO first; RTK / GNSS integration comes only after VIO is proven.

## Safety boundary

| | |
|---|---|
| Publishes | `/vio/odom`, `/vio/status` (+ `/tf` only when `publish_tf:=true`, used in replays) |
| Never publishes | `/robot/odom`, any `/cmd_vel*` (refused in `vio_odom_adapter`) |
| Touches | nothing in `PPBv2_Navigation/` or `PPBv2_Imaging/` |
| DDS | `ROS_LOCALHOST_ONLY=1` in the container; nothing reaches the robot network |

## Topic contract

| Topic | Type | Notes |
|---|---|---|
| `/vio/cam0/image_raw`, `/vio/cam1/image_raw` | `sensor_msgs/Image` | mono8, header stamp = exposure time, stereo stamps identical |
| `/vio/cam0/camera_info`, `/vio/cam1/camera_info` | `sensor_msgs/CameraInfo` | from real calibration only |
| `/vio/imu` | `sensor_msgs/Imu` | raw gyro [rad/s] + accel [m/s²], `frame_id: imu_link`, `orientation_covariance[0] = -1` |
| `/vio/odom` | `nav_msgs/Odometry` | `vio_world` → `base_link`; twist in `base_link` |
| `/vio/status` | `diagnostic_msgs/DiagnosticArray` | estimator output + input stream health |

`vio_world` is the estimator's own gravity-aligned start frame. It is **not**
the GNSS `map` frame; aligning the two is a later phase.

## What has been done

### 1. Development environment

- `docker/Dockerfile`: ROS 2 Humble (`ros:humble-perception-jammy`, pinned by
  digest) with MINS `e245da6` and OpenVINS `6948812` built from source at
  pinned commits. They are built into **separate workspaces**
  (`/opt/vio/mins_ws`, `/opt/vio/openvins_ws`) because MINS bundles its own copy
  of OpenVINS' `ov_core` and the package names collide.
- Python evaluation tools (`evo`, `rosbags`, …) in their own virtualenv
  (`/opt/vio-tools`), installed from a lock file with `pip check`, isolated
  from ROS' Python.
- `docker/compose.yaml`: mounts this folder at `/ppbv2_vio` and `data/` at
  `/data`, keeps DDS on localhost, exposes the Foxglove port.

### 2. ROS 2 package `ppb_vio` (`src/ppb_vio/`)

- **`vio_odom_adapter`**: the only place estimator output enters the PPBv2
  topic contract. It converts the estimator's IMU-frame odometry to
  `base_link` (pose, velocity, and covariance, using the `base_link → imu_link`
  mounting), rejects malformed output (non-finite values, bad quaternions,
  invalid or non-increasing stamps) instead of crashing, publishes
  `/vio/odom` and a health status, and refuses to publish on `/robot/odom` or
  any `/cmd_vel*` topic.
- **`vio_stream_monitor`**: checks the camera and IMU inputs (rate, gaps,
  timestamp order, stereo pairing, CameraInfo, IMU units) and reports on
  `/vio/status`. It reads only image headers and uses the same QoS as the
  estimators, so it never reports its own dropped messages as input faults.
- **`launch/vio.launch.py`**: starts one estimator + adapter + monitor
  (+ recorder, + optional Foxglove bridge). It builds the estimator config for
  each run (see design notes), refuses to reuse an output folder, and shuts the
  whole run down if the estimator or adapter dies.
- **Profiles**: `config/estimators/` (which upstream config, which keys PPBv2
  overrides, which frame each estimator reports velocity in) and
  `config/platforms/` (frames, IMU mounting, expected rates).

### 3. Offline tools (`tools/`, `scripts/`)

- `tools/prepare_asl.py`: converts an ASL-format dataset (EuRoC, TUM-VI) into a
  ROS 2 bag on the `/vio/*` topics, with validated timestamps, real
  CameraInfo, and a conversion report. Writes atomically.
- `scripts/run_replay.sh`: one command per test. It starts the pipeline, waits
  until the estimator and monitor are subscribed, replays the bag, stops
  everything cleanly, checks for crashes, and scores the run.
- `tools/evaluate_run.py`: scores `/vio/odom` against ground truth (APE after
  position + yaw alignment, drift per 10 m, start-up time, coverage) and marks
  a run INVALID when the recorded input health shows the machine could not
  deliver the data on time.
- `tools/make_demo_mcap.py`: packages a passing run into one MCAP file for
  Foxglove (images, `/vio/odom`, trail, aligned ground truth).

### 4. Bugs found and fixed

In upstream MINS (fixed by patches applied at build time; the build fails if a
patch stops applying):

- `0001-ros2-output-timestamps`: odometry was stamped with
  `rclcpp::Time(double_seconds)`, which C++ treats as nanoseconds, so stamps
  came out ~10⁹ times too small.
- `0002-camera-compressed-default`: the optional `camN.compressed` setting was
  read into an uninitialized `bool`. With the stock configs MINS could
  subscribe to compressed images and never receive a frame (this happened in
  our first MINS run).

In our own first version (found by real runs and by a full code review):

- Replays could start before the estimator subscribed, losing the first
  seconds it initializes from → now waits for all subscriptions.
- Nodes exited with errors / segfaulted on Ctrl-C (ROS Humble raises
  `ExternalShutdownException`), and background jobs ignored SIGINT so the
  pipeline could not be stopped cleanly → fixed and checked on every run.
- The stream monitor dropped messages itself (best-effort QoS, full image
  deserialization) and reported that as bad input → reliable QoS, header-only
  parsing.
- Accuracy was scored after a full SE(3) alignment, which can hide roll/pitch
  errors → now position + yaw only (the standard for VIO).
- A malformed timestamp could crash the adapter; `publish_tf` defaulted to on
  (would give `base_link` a second parent on the robot); a failed dataset
  conversion blocked retries; missing package dependencies; README errors.

### 5. Verification

Replays at 0.5x speed on a MacBook (8 GB, Docker).

**EuRoC `V1_01_easy`** (58 m, stereo 752x480 @ 20 Hz, IMU 200 Hz, upstream
`config/euroc_mav` for both estimators):

| Estimator | Result | APE RMSE (max) | Drift / 10 m | Start-up | Coverage |
|---|---|---|---|---|---|
| MINS (camera + IMU) | **PASS**; 2 runs: 5.7 and 6.3 cm | 5.7 cm (17.7 cm) | 6.3 cm, 1.4° | 5.3 s | 100 % |
| OpenVINS (reference) | **PASS** | 3.9 cm (7.9 cm) | 5.9 cm, 0.8° | 5.6 s | 100 % |

**TUM-VI `dataset-room1_512_16`** (146 m, stereo fisheye 512x512 @ 20 Hz):

| Estimator | Result | APE RMSE |
|---|---|---|
| OpenVINS, upstream `config/tum_vi` | **PASS**, 4 runs | 7.1–7.8 cm |
| MINS, upstream `config/tum_vi` | **FAIL**, 2 of 2 runs | 16.6 m / 2.0 km |

What this establishes:

- The pipeline is correct end to end for both estimators: topics, estimator
  configs, frames, timestamps, `base_link` conversion, recording, and scoring
  all agree with ground truth to centimetres. Position+yaw-aligned and
  SE(3)-aligned errors agree, confirming the ground truth is gravity-aligned.
- The published `/vio/odom` velocity is right for both estimators (they report
  velocity in different frames): rotated to the world frame it matches the
  differentiated position to 1.7 cm/s (OpenVINS) and 2.4 cm/s (MINS) at
  ~1.1 m/s, versus ~1 m/s if the frame were wrong.
- MINS works, repeatably, with its maintained EuRoC config. Its TUM-VI config
  (unchanged since MINS' first commit; no fisheye image mask, camera
  calibration re-estimated online with loose limits) is not usable: with
  complete input it rejected nearly all camera updates (consistency-check pass
  rate ~20 % vs ~83 % when healthy) and drifted on IMU alone. **Our camera's
  MINS config has to be written and validated by us.**
- MINS is sensitive to dropped input: in a replay where the overloaded laptop
  delivered ~17 instead of 20 camera frames/s for stretches, MINS diverged
  completely (the run was correctly reported INVALID). OpenVINS tolerated
  comparable conditions earlier.

Plots: `data/demo/trajectories.png` (generated, git-ignored).

## Changes to the repository

| Path | Change |
|---|---|
| `PPBv2_VIO/` | **new**: everything described above (46 tracked files) |
| `README.md` (repository root) | 4-line "PPBv2 VIO (in development)" section linking here |
| `PPBv2_Navigation/`, `PPBv2_Imaging/` | unchanged |

```text
PPBv2_VIO/
├── README.md
├── .dockerignore, .gitignore
├── docker/
│   ├── Dockerfile                    pinned base image, MINS, OpenVINS
│   ├── compose.yaml
│   ├── requirements-tools.txt        top-level tool pins
│   └── requirements-tools.lock.txt   fully resolved tool versions (installed)
├── patches/mins/
│   ├── 0001-ros2-output-timestamps.patch
│   └── 0002-camera-compressed-default.patch
├── scripts/
│   ├── vio_env.sh                    source: sets up a shell for one estimator
│   ├── build_ppb_vio.sh              build ppb_vio + run all unit tests
│   └── run_replay.sh                 replay → estimator → /vio/odom → score
├── src/ppb_vio/                      ROS 2 package (ament_python)
│   ├── ppb_vio/vio_odom_adapter.py
│   ├── ppb_vio/vio_stream_monitor.py
│   ├── ppb_vio/lib/                  pure logic, no ROS imports:
│   │   ├── odom_conversion.py        IMU → base_link pose/velocity/covariance
│   │   ├── transforms.py             rotations, quaternions
│   │   ├── stream_health.py          rate / gap / order / stereo / IMU checks
│   │   ├── cdr_image_header.py       read image headers without pixels
│   │   ├── estimator_config.py       strict per-run config overrides
│   │   └── time_utils.py
│   ├── config/estimators/            mins_euroc, mins_tumvi (not validated),
│   │                                 openvins_euroc, openvins_tumvi
│   ├── config/platforms/             euroc, tumvi
│   ├── launch/vio.launch.py
│   └── test/                         49 tests
├── tools/
│   ├── prepare_asl.py                dataset → ROS 2 bag
│   ├── evaluate_run.py               score a run
│   ├── make_demo_mcap.py             demo file for Foxglove
│   ├── acceptance.yaml               pass/fail limits per sequence
│   ├── ppb_vio_tools/                euroc.py, images.py, metrics.py
│   └── test/                         19 tests
└── data/                             datasets, runs, demo files (git-ignored)
```

## How to use

### Build (first time ~45–60 min on an 8 GB Mac)

Docker Desktop needs 6 GB memory and 4 GB swap **for the build** (Settings →
Resources).

```bash
cd PPBv2_VIO
docker compose -f docker/compose.yaml build
docker compose -f docker/compose.yaml run --rm vio bash

# inside the container
/ppbv2_vio/scripts/build_ppb_vio.sh                  # build + all unit tests
```

### Get a dataset (goes to `PPBv2_VIO/data/`, git-ignored)

TUM-VI room sequences (stereo fisheye 20 Hz, IMU 200 Hz, full motion-capture
ground truth):

```bash
# on the host
mkdir -p data/tumvi/dataset-room1_512_16 && cd data/tumvi/dataset-room1_512_16
curl -fLO https://cdn3.vision.in.tum.de/tumvi/exported/euroc/512_16/dataset-room1_512_16.tar
tar -xf dataset-room1_512_16.tar
```

EuRoC MAV: ETH distributes it only as per-room bundles (`vicon_room1.zip`,
6.0 GB) from the
[ETH Research Collection](https://www.research-collection.ethz.ch/handle/20.500.11850/690084),
which rate-limits scripted downloads, so use a browser. Each bundle holds, per
sequence, a ROS 1 `.bag` and an ASL `.zip`; only the ASL zip is used:

```bash
# on the host (Safari may already have unpacked the outer bundle)
Z=~/Downloads/vicon_room1/V1_01_easy/V1_01_easy.zip
unzip -tq "$Z"                                       # CRC check before use
mkdir -p data/euroc/V1_01_easy/V1_01_easy
unzip -q "$Z" -d data/euroc/V1_01_easy/V1_01_easy    # -> .../V1_01_easy/mav0/...
```

Convert to a ROS 2 bag (inside the container). TUM-VI ships no `sensor.yaml`,
so its calibration is read from the Kalibr camchain in the pinned OpenVINS
checkout; EuRoC needs no `--camchain`.

```bash
/ppbv2_vio/tools/prepare_asl.py /data/tumvi/dataset-room1_512_16/dataset-room1_512_16 \
    /data/tumvi/dataset-room1_512_16/ros2_bag \
    --camchain /opt/vio/openvins_ws/src/open_vins/config/tum_vi/kalibr_imucam_chain.yaml
/ppbv2_vio/tools/prepare_asl.py /data/euroc/V1_01_easy/V1_01_easy /data/euroc/V1_01_easy/ros2_bag
```

### Run and score

Before replays on an 8 GB Mac: set Docker memory to **4 GB** (a run uses
< 1 GB; 6 GB starves macOS), close browsers and other heavy apps, and run
`docker desktop restart`.

```bash
/ppbv2_vio/scripts/run_replay.sh mins     euroc V1_01_easy --rate 0.5
/ppbv2_vio/scripts/run_replay.sh openvins euroc V1_01_easy --rate 0.5
/ppbv2_vio/scripts/run_replay.sh openvins tumvi dataset-room1_512_16 --rate 0.5
```

Each run writes `/data/runs/<time>_<estimator>_<sequence>/` with the exact
estimator config used, the recorded output bag, logs, `vio_odom.tum`,
`evaluation.json`, and `summary.txt`. Exit codes: 0 pass, 1 fail (accuracy, a
crash, or an unclean shutdown), 3 invalid (inputs were degraded, so the result
says nothing about the estimator; rerun on a less loaded machine).

### Demo in Foxglove

Live replays on a laptop depend on machine load, so for demos use a file made
from a run that already passed. Nothing is computed during playback:

```bash
/ppbv2_vio/tools/make_demo_mcap.py --run-dir /data/runs/<passing run> \
    --sensor-bag /data/euroc/V1_01_easy/ros2_bag \
    --gt /opt/vio/openvins_ws/src/open_vins/ov_data/euroc_mav/V1_01_easy.txt \
    --out /data/demo/V1_01_easy_mins.mcap
```

In the **Foxglove app** (not a browser): *Open local file* → the `.mcap`. Add an
Image panel for `/vio/cam0/image_raw` and a 3D panel; in the 3D panel's
settings set **Display frame = `vio_world`** (Foxglove may default to
`cam0_optical_frame`, which shows an empty grid) and enable
`/ground_truth/path`, `/vio/path`, `/vio/odom`. Then press play.

Live view instead: add `--foxglove` to `run_replay.sh`, start the container
with `docker compose -f docker/compose.yaml run --rm --service-ports vio ...`,
and in Foxglove choose *Open connection → Foxglove WebSocket →
`ws://localhost:8765`*. `ws://` addresses do not open in Safari.

## Design notes

- **Two workspaces.** MINS bundles OpenVINS' `ov_core`; never source both
  workspaces in one shell (`vio_env.sh` refuses).
- **No vendored GPL configs.** MINS and OpenVINS are GPL-3; this repository is
  MIT. Estimator configs are read from the pinned install, the overrides in
  `config/estimators/*.yaml` are applied (each must match exactly one line or
  the run is refused), and the result is written into the run folder.
- **Camera + IMU only, enforced.** MINS enables a sensor iff its
  `config_<sensor>.yaml` exists; the MINS profiles list wheel/GNSS/LiDAR/Vicon
  files as forbidden, and the launch refuses to start if any is present.
- **Estimator-specific velocity frames.** MINS reports linear velocity in its
  world frame, OpenVINS in the IMU frame. Each profile states which, and the
  adapter has no default for it (checked against real recordings, see above).
- **Covariance.** Both estimators report pose covariance as [position error in
  world, orientation error in IMU frame]. The adapter maps it to `base_link` to
  first order; unit tests check the mapping against Monte Carlo sampling.
- **Pinned everything.** Base image by digest, estimators by commit, Python
  tools by lock file.

## Known issues and limitations

- **MINS fisheye config:** the upstream TUM-VI config diverges (see above); no
  validated MINS config for a wide-angle camera exists yet.
- **MINS and dropped frames:** MINS diverged when the laptop could not deliver
  every frame; needs checking on the Orin under real camera load.
- **OpenVINS segfaults on shutdown** under Humble (known upstream). All output
  is recorded by then; the runner reports it as a warning, not a failure.
- **8 GB Mac:** builds need 6 GB for Docker, replays starve macOS at 6 GB (use
  4 GB). After long builds containers can hang in `Created`; `docker desktop
  restart` fixes it.
- **Disk use:** the Docker image and build cache take ~25 GB (~11 GB
  reclaimable with `docker builder prune` and `docker image prune`); datasets
  and runs ~10 GB in `data/` (delete `data/runs` and the TUM-VI `.tar` to reclaim space).
- **Demo file:** the camera frame (`cam0_optical_frame`) is not connected to
  the trajectory in TF, so the 3D panel must be set to `vio_world`.
- **EuRoC download:** ETH rate-limits scripted access; download in a browser.

## What needs to be done

Before the camera arrives (no hardware needed):

1. **Write and validate a PPBv2 MINS config for a wide-angle stereo camera**
   (image mask for the fisheye border, camera calibration held fixed instead
   of re-estimated online). Validate on TUM-VI `room1` until it passes as
   reliably as OpenVINS. When the real camera arrives, only its calibration
   values change.
2. **Run `V1_02_medium` and `V1_03_difficult`** (already downloaded with
   `vicon_room1`) to see where MINS and OpenVINS start to fail under faster
   motion.
3. **Commit** this work on `setup-vio` and open a PR once reviewed.
4. Optional: add the camera mounting to the demo file's TF so the camera frame
   moves with the trajectory in Foxglove.

With hardware:

5. **Choose and mount the stereo camera** with onboard IMU; write
   `config/platforms/ppbv2.yaml` (measured `base_link → imu_link`, rates) and a
   driver launch that publishes on `/vio/*`.
6. **Calibrate with Kalibr**: camera intrinsics, camera–IMU extrinsics, and
   camera–IMU time offset; put the results into our MINS / OpenVINS configs.
7. **Build the image on the Orin** (`BUILD_JOBS=6`), rerun the dataset tests
   there, and check MINS under real camera load.
8. **Record on the robot** (straight line, 90° turn, loop, orchard row) with
   `/robot/odom` from RTK as the reference, and score with the same tools.

Only after VIO is proven on the robot:

9. Send `/vio/odom` and `/vio/status` from the Orin (Humble) to the Pi (Jazzy)
   through a narrow DDS bridge, and test that link explicitly.
10. Wheel odometry, GNSS fusion in MINS, a confidence-aware safety layer, and
    visual row-following.
