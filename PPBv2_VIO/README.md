# PPBv2 VIO

Stereo visual-inertial odometry for the **PPBv2 Amiga**, running **OpenVINS on the Jetson AGX
Orin** (ROS 2 Humble). It is the first layer of the localization stack in
`PPBv2_Perception_Design.pdf` (repository root): VIO is one sensor behind a fixed
`/vio/odom` (+ covariance) and `/vio/status` contract, later fused with wheel odometry and
RTK by `robot_localization`, so the robot keeps localizing under canopy where RTK fails.

This folder is Stage 1 of that design (CPU only; GPU work comes after Stage 1 is proven on
the robot). Everything runs in Docker (arm64, the Orin's distro). Nothing here modifies
`PPBv2_Navigation/` or `PPBv2_Imaging/`, and nothing publishes `/robot/odom` or `/cmd_vel*`.

## Status (2026-10-26)

| Step toward the robot | State |
|---|---|
| OpenVINS (pinned `69488123`) in the arm64 Humble image; Orin overlay `docker/compose.robot.yaml` | done (Orin build not yet run) |
| `/vio/odom` adapter: IMU → `base_link`, covariance scaling, divergence health gate on `/vio/status` | done |
| PPBv2 OpenVINS profile (`openvins_ppbv2`) with ground-vehicle settings learned on field data | done |
| Calibration path: Kalibr + Allan variance → OpenVINS config + platform file (`kalibr_to_openvins.py`) | done, needs the robot's calibration |
| Live entry point on the Orin (`run_robot_vio.sh`) | done, dry-run with a replayed bag |
| Robot recording → scored dataset (`prepare_ppbv2.py`, RTK as ground truth) | done, needs robot recordings |
| Offline bench, stress suite, evaluation harness (EuRoC, Rosario v1 field data) | done |
| **Hardware**: navigation camera + IMU on the Amiga, its ROS 2 driver, calibration, recordings | not started (spec Tasks 5–7) |
| Measure OpenVINS compute on the Orin (`run_bench.sh` there) | not started (needs the Orin) |
| Fusion with wheel odometry + RTK (spec Tasks 3–4), live integration (Task 8) | not started |

## On the robot

### 1. Navigation camera driver: the `/vio/*` contract

The driver (chosen with the hardware) must publish:

| Topic | Type | Requirement |
|---|---|---|
| `/vio/cam0/image_raw`, `/vio/cam1/image_raw` | `sensor_msgs/Image` | mono8, stamp = exposure time, **identical stamps for a stereo pair** (hardware trigger) |
| `/vio/cam0/camera_info`, `/vio/cam1/camera_info` | `sensor_msgs/CameraInfo` | same resolution as the calibration |
| `/vio/imu` | `sensor_msgs/Imu` | raw gyro (rad/s) + accel (m/s²), `frame_id: imu_link`, `orientation_covariance[0] = -1`, ≥ ±16 g range, hardware-synchronized stamps |

### 2. Calibrate (spec Task 5)

Kalibr (stereo + camera–IMU) and Allan variance (IMU noise), then, with the IMU mounting
measured on the Amiga relative to `base_link` (the point `/robot/odom` reports: the
dual-antenna midpoint, `PPBv2_Navigation/HANDOFF.md`):

```bash
tools/kalibr_to_openvins.py --camchain camchain-imucam.yaml --imu imu.yaml \
    --imu-in-base-xyz X Y Z --imu-in-base-rpy-deg R P Y --camera-hz 30 \
    --out /data/calibration/ppbv2_<date>
```

Writes OpenVINS calibration files, `platform.yaml` (mounting, Amiga health limits: 2.5 m/s
horizontal, 0.5 m/s vertical, 4 m/s²; the navigation stack caps commands at 2.0 m/s), and
a report. It checks that the stereo pair is hardware-synchronized and that Kalibr's
camera–IMU extrinsics agree with its stereo extrinsic. A 0.29° disagreement in the
Rosario dataset's calibration made stereo VIO 11 % short; the tool flags and corrects it.

### 3. Run live on the Orin

```bash
cd PPBv2_VIO
docker compose -f docker/compose.yaml -f docker/compose.robot.yaml build      # BUILD_JOBS=6
docker compose -f docker/compose.yaml -f docker/compose.robot.yaml run --rm vio bash
/ppbv2_vio/scripts/build_ppb_vio.sh
/ppbv2_vio/scripts/run_robot_vio.sh --calibration /data/calibration/ppbv2_<date>
```

It refuses to start without a complete calibration, waits for the camera driver, and
records `/vio/odom` + `/vio/status` to `/data/robot_runs/`. DDS is localhost-only on domain 42
(imaging uses 77, navigation the default): nothing reaches the Pi 5 or the motors until
live integration (spec Task 8).

### 4. Record and evaluate (spec Task 7)

Record the camera, IMU and RTK pose together (clocks of the Orin and the Pi 5 synchronized
with chrony/PTP): `/vio/cam{0,1}/image_raw`, `/vio/cam{0,1}/camera_info`, `/vio/imu`,
`/robot/odom`, `/gps/fix`, `/gps/rtk_status_flag` (and `/wheel/odom` once it exists). Then:

```bash
tools/prepare_ppbv2.py /data/recordings/<bag> /data/calibration/ppbv2_<date> /data/ppbv2/<name>
scripts/run_bench.sh openvins ppbv2 <name>              # score against RTK
scripts/run_stress_suite.sh openvins ppbv2 <name>       # frame drop, jitter, blackout, ...
```

Run the bench on the Orin itself to get its real compute budget (per-update processing
time vs the camera period, reported by `evaluate_run.py`).

## Evidence before the hardware (datasets)

OpenVINS, offline bench (same code path as on the robot). ATE: position + yaw aligned, at
the point the ground truth tracks.

| Data | Result |
|---|---|
| EuRoC V1_01_easy (58 m, sanity) | 4.3 cm (live replay 3.9 cm: the bench is faithful) |
| Rosario v1 seq02 (320 m soybean field), wheel odometry alone | 4.42 m |
| Rosario v1 seq02, OpenVINS with the PPBv2 settings | **3.03 m** (healthy rows: 1.48 m over 139 m) |
| Stress (10 % frame drop, 5 ms jitter, 2 s blackout, 2 % IMU drop, mid-row start) | all initialize in 2.4 s, ~100 % coverage, ATE 1.9–3.1 m |
| Compute on this laptop (1 thread, 672×376) | 6.3 ms median / 8.9 ms p95 per update vs 65 ms frame period |

Lessons carried into the PPBv2 profile and tools:

1. **Hold the camera calibration fixed on a ground vehicle.** Online extrinsic calibration
   drifted the stereo baseline 0.120 → 0.170 m (scale ×1.43); planar motion cannot
   constrain it. (Conflicts with the spec's "seed online refinement", Requirement 6.)
2. **Check the stereo calibration's consistency** (see step 2 above).
3. **Initialization under motion** is supported (dynamic init, 5° threshold) for restarts
   while driving; Rosario's robot never stood still and VIO started in 2.4 s.
4. **Headland U-turns are where VIO fails** (slow, 180° turns): it loses height there in
   every run. The health gate flags it 2–3 s after onset, so fusion can drop VIO and coast
   on wheel odometry (Requirement 4).
5. **OpenVINS is overconfident on field data** (position NEES 44 vs 3 on Rosario). The
   adapter's `covariance_scale` must be calibrated from robot recordings before fusion
   (Requirement 5; `evaluate_run.py` reports a suggested value).
6. **Monocular is not an option**: at constant speed the scale is unobservable
   (Requirement 3). Stereo is required.
7. **Live replays on a loaded laptop drop IMU messages** and diverge the filter; the
   harness marks such runs INVALID. Scoring uses the offline bench.

Figures: `data/plots/` (`tools/plot_trajectories.py`). Tables: `data/bench/`
(`tools/bench_summary.py`).

## Layout

```text
PPBv2_VIO/
├── docker/   Dockerfile (Humble + OpenVINS + tools), compose.yaml (laptop), compose.robot.yaml (Orin)
├── scripts/
│   ├── run_robot_vio.sh      OpenVINS live on the robot from a calibration folder
│   ├── run_bench.sh          offline (serial) run on a dataset or robot recording -> scored folder
│   ├── run_stress_suite.sh   stressed copies of a recording through the bench
│   ├── run_replay.sh         live ROS replay (end-to-end check of the nodes)
│   ├── run_pre_gpu.sh        whole pre-GPU check in one command
│   ├── build_ppb_vio.sh, vio_env.sh, fetch_rosario_v1.sh
├── src/
│   ├── ppb_vio/              adapter (+ health gate, covariance scale), stream monitor, launch,
│   │                         profiles (config/estimators/openvins_ppbv2.yaml), pure libs (lib/)
│   └── ppb_vio_openvins/     openvins_serial: offline OpenVINS runner for the bench
└── tools/                    kalibr_to_openvins, prepare_ppbv2, dataset loaders, evaluation,
                              offline adapter, stress bags, plots (see tools/README.md)
```

## Safety boundary

| | |
|---|---|
| Publishes | `/vio/odom`, `/vio/status` (+ `/tf` only with `publish_tf:=true`, replays only) |
| Never publishes | `/robot/odom`, any `/cmd_vel*` (refused in `vio_odom_adapter`) |
| DDS | localhost-only (laptop and robot) until live integration is designed (Task 8) |

`/vio/odom`: `vio_world` → `base_link`, pose covariance [position in world, orientation in
base], twist in `base_link`. `vio_world` is the estimator's own gravity-aligned start frame,
not the GNSS `map` frame. `/vio/status`: `ppb_vio: odometry` (liveness), `ppb_vio: vio health`
(divergence gate; latches ERROR until restart), input stream checks.

## Development on a laptop

```bash
cd PPBv2_VIO
docker compose -f docker/compose.yaml build && docker compose -f docker/compose.yaml run --rm vio bash
/ppbv2_vio/scripts/build_ppb_vio.sh                    # build + all unit tests
/ppbv2_vio/scripts/run_bench.sh openvins rosario_v1 sequence02
```

Or `scripts/run_pre_gpu.sh` from the host. Datasets and loaders: [tools/README.md](tools/README.md).
Pinned everything (base image digest, OpenVINS commit, Python lock file); upstream OpenVINS
configs are read from the pinned install and only documented overrides are applied per run.

## Next (toward the robot)

1. Choose and mount the navigation camera + IMU; ROS 2 driver publishing the `/vio/*` contract.
2. Calibrate (Kalibr, Allan variance) → `kalibr_to_openvins.py`.
3. Build the image on the Orin; run the bench there for the real compute budget.
4. Hand-pushed, then driven recordings in open sky and under canopy → `prepare_ppbv2.py` → bench.
5. Calibrate `covariance_scale` from those recordings; then fusion (spec Tasks 3–4) and live
   integration (Task 8).
