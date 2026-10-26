# Offline tools: dataset loaders and evaluation harness

Everything here runs in the `ppbv2-vio:humble` container with the isolated tools
Python (`/opt/vio-tools/bin/python -I`). No ROS runtime is needed except to replay bags.

```bash
cd PPBv2_VIO
docker compose -f docker/compose.yaml run --rm vio bash      # PPBv2_VIO -> /ppbv2_vio, data/ -> /data
```

## Dataset loaders

Each loader writes a ROS 2 bag on the PPBv2 VIO topic contract (`/vio/cam{0,1}/image_raw`
mono8, `/vio/cam{0,1}/camera_info`, `/vio/imu`) plus, next to it, `ground_truth.tum`,
`dataset.yaml` (ground-truth conventions for the evaluator) and `conversion_report.json`.

| Dataset | Loader | Role |
|---|---|---|
| EuRoC MAV `V1_01_easy` | `prepare_asl.py` | pipeline sanity (frames, time) |
| Rosario v1, sequence 1-6 | `prepare_rosario_v1.py` | primary: soybean field, ground vehicle |
| Rosario v2 | `ppb_vio_tools/rosario_v2.py` | stub, deferred to the fusion phase |

### EuRoC (already converted on this machine)

```bash
# Conversion (see the top-level README for getting the ASL zip):
tools/prepare_asl.py /data/euroc/V1_01_easy/V1_01_easy /data/euroc/V1_01_easy/ros2_bag \
    --ground-truth /opt/vio/openvins_ws/src/open_vins/ov_data/euroc_mav/V1_01_easy.txt
# For a bag converted before dataset.yaml existed:
tools/prepare_asl.py /data/euroc/V1_01_easy/V1_01_easy /data/euroc/V1_01_easy/ros2_bag \
    --descriptor-only --sequence V1_01_easy \
    --ground-truth /opt/vio/openvins_ws/src/open_vins/ov_data/euroc_mav/V1_01_easy.txt
```

### Rosario v1

```bash
# On the host (or in the container): ~2.7 GB for sequence 2, resumable.
scripts/fetch_rosario_v1.sh 2
# In the container: join parts + check the published MD5, convert ROS 1 -> ROS 2.
tools/prepare_rosario_v1.py 2 /data/rosario_v1/sequence02/download /data/rosario_v1/sequence02 \
    --delete-parts
```

What the Rosario v1 loader does and checks:

- Joins the split bag and verifies the MD5 published by CIFASIS; a mismatch leaves nothing behind.
- `/stereo/{left,right}/image_raw` (bgr8) become mono8 with OpenCV's exact `BGR2GRAY`
  arithmetic (what `cv_bridge` gives an estimator asking for mono8). Camera info comes from
  `calibrationNN.yaml` (Kalibr), not from the bag.
- `/imu` -> `/vio/imu` (frame `imu_link`, orientation marked unknown). The IMU must measure
  gravity in m/s^2 (median |a| in 9.3-10.3), else conversion is refused.
- `/odom` -> `/wheel/odom` unchanged. The report states whether its twist is body-frame
  (`wheel_odom_twist_frame_check`).
- `/tf_static` from the calibration file: `base_link -> imu_link -> cam{0,1}_optical_frame`,
  `base_link -> gps`. `rotation_imu_baselink` is R_base_imu (agrees with the bag's own
  tf_static and with gravity).
- Ground truth is the **GPS antenna position** (no orientation). `dataset.yaml` records the
  antenna's position in `base_link`; the evaluator scores the estimate at that point.
- Refuses on missing/mistyped topics, IMU or camera stamps going backwards, or > 1 % unpaired
  stereo frames.

Disk: the joined ROS 1 bag (2.7 GB) is kept as the source unless `--delete-source` is passed;
the ROS 2 bag is ~2 GB (mono8, uncompressed).

## Evaluation harness: `evaluate_run.py`

```bash
# A replay recorded by scripts/run_replay.sh (scores /vio/odom in <run>/record):
tools/evaluate_run.py --run-dir /data/runs/<run> --dataset /data/euroc/V1_01_easy/dataset.yaml \
    --acceptance tools/acceptance.yaml
# Any odometry topic in any bag, report only (no acceptance limits):
tools/evaluate_run.py --run-dir /data/runs/wheel_seq02 \
    --estimate-bag /data/rosario_v1/sequence02/ros2_bag --topic /wheel/odom \
    --dataset /data/rosario_v1/sequence02/dataset.yaml
```

Outputs `estimate.tum` and `evaluation.json` in `--run-dir`.

| Metric | Definition |
|---|---|
| ATE | RMSE / max position error after position + yaw alignment (no scale), at the ground-truth point; also horizontal-only and SE(3)-aligned variants |
| RTE | RMSE of translation error over consecutive 10 m segments of the aligned estimate (positions only); rotation error per 10 m only if the ground truth has orientation |
| NEES | position NEES from `/vio/odom` covariance (position block mapped to the ground-truth point). **Stub: reported, never gated**, until Task 2 verifies each frontend's covariance |
| availability | init latency, coverage, longest output gap |
| input health | `/vio/status` stream diagnostics (only when scoring `/vio/odom`); degraded inputs make the run INVALID (exit 3) |

Estimates are interpolated at ground-truth stamps, never across a gap longer than
`--max-bracket` (0.25 s).

## Tests

```bash
cd /ppbv2_vio/tools && /opt/vio-tools/bin/python -I -m pytest -q -p no:cacheprovider test
```

`test/test_harness.py` covers association, the reference-point covariance (against Monte
Carlo), RTE, NEES, and an end-to-end Rosario v1 conversion of a synthetic ROS 1 bag
(including refusal on wrong MD5 and non-SI IMU).
