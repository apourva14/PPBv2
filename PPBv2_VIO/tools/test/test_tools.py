from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ppb_vio_tools.euroc import (  # noqa: E402
    DatasetError, find_mav0, read_camera_calibration, read_camera_index, read_imu,
    read_kalibr_camchain)
from ppb_vio_tools.metrics import (  # noqa: E402
    align_position_yaw, availability, check_acceptance, input_health,
    position_errors_after_yaw_alignment)

SENSOR_YAML = """%YAML:1.0
sensor_type: camera
comment: VI-Sensor cam0 (MT9M034)
T_BS:
  cols: 4
  rows: 4
  data: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
rate_hz: 20
resolution: [752, 480]
camera_model: pinhole
intrinsics: [458.654, 457.296, 367.215, 248.375]
distortion_model: radial-tangential
distortion_coefficients: [-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05]
"""


def make_asl(root, cam_stamps=(100, 200, 300), imu_stamps=(90, 95, 100)):
    mav0 = root / 'seq' / 'mav0'
    for cam in ('cam0', 'cam1'):
        d = mav0 / cam / 'data'
        d.mkdir(parents=True)
        (mav0 / cam / 'sensor.yaml').write_text(SENSOR_YAML)
        lines = ['#timestamp [ns],filename']
        for s in cam_stamps:
            (d / f'{s}.png').write_bytes(b'')
            lines.append(f'{s},{s}.png')
        (mav0 / cam / 'data.csv').write_text('\n'.join(lines) + '\n')
    (mav0 / 'imu0').mkdir()
    rows = ['#timestamp [ns],w_x,w_y,w_z,a_x,a_y,a_z']
    rows += [f'{s},0.1,0.2,0.3,0.0,0.0,9.81' for s in imu_stamps]
    (mav0 / 'imu0' / 'data.csv').write_text('\n'.join(rows) + '\n')
    return root / 'seq'


def test_reads_valid_asl_sequence(tmp_path):
    seq = make_asl(tmp_path)
    mav0 = find_mav0(seq)
    assert find_mav0(mav0) == mav0
    index = read_camera_index(mav0 / 'cam0')
    assert [s for s, _ in index] == [100, 200, 300]
    imu = read_imu(mav0 / 'imu0')
    assert imu[0] == (90, (0.1, 0.2, 0.3), (0.0, 0.0, 9.81))


def test_camera_calibration_and_camera_info(tmp_path):
    seq = make_asl(tmp_path)
    calib = read_camera_calibration(seq / 'mav0' / 'cam0' / 'sensor.yaml')
    info = calib.camera_info_fields()
    assert (info['width'], info['height']) == (752, 480)
    assert info['k'] == [458.654, 0.0, 367.215, 0.0, 457.296, 248.375, 0.0, 0.0, 1.0]
    assert info['distortion_model'] == 'plumb_bob' and len(info['d']) == 5
    assert info['p'][3] == 0.0 and info['p'][10] == 1.0


def test_equidistant_camera_info(tmp_path):
    seq = make_asl(tmp_path)
    path = seq / 'mav0' / 'cam0' / 'sensor.yaml'
    path.write_text(SENSOR_YAML.replace('radial-tangential', 'equidistant'))
    info = read_camera_calibration(path).camera_info_fields()
    assert info['distortion_model'] == 'equidistant' and len(info['d']) == 4


def test_rejects_unknown_distortion_model(tmp_path):
    seq = make_asl(tmp_path)
    path = seq / 'mav0' / 'cam0' / 'sensor.yaml'
    path.write_text(SENSOR_YAML.replace('radial-tangential', 'omni'))
    with pytest.raises(DatasetError, match='distortion_model'):
        read_camera_calibration(path)


CAMCHAIN = """%YAML:1.0 # need to specify the file type at the top!

cam0:
  T_cam_imu:
    - [1.0, 0.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0, 0.0]
    - [0.0, 0.0, 1.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
  camera_model: pinhole
  distortion_coeffs: [0.0034, 0.0007, -0.0020, 0.0002]
  distortion_model: equidistant
  intrinsics: [190.97, 190.97, 254.93, 256.89]
  resolution: [512, 512]
  rostopic: /cam0/image_raw
"""


def test_reads_kalibr_camchain(tmp_path):
    path = tmp_path / 'camchain.yaml'
    path.write_text(CAMCHAIN)
    calib = read_kalibr_camchain(path, 'cam0')
    assert (calib.width, calib.fx, calib.distortion_model) == (512, 190.97, 'equidistant')
    assert calib.rate_hz is None
    path.write_text(CAMCHAIN.replace('equidistant', 'radtan'))
    assert read_kalibr_camchain(path, 'cam0').camera_info_fields()['distortion_model'] == 'plumb_bob'
    with pytest.raises(DatasetError, match='no cam1'):
        read_kalibr_camchain(path, 'cam1')


def test_rejects_non_increasing_timestamps(tmp_path):
    seq = make_asl(tmp_path, imu_stamps=(90, 90, 100))
    with pytest.raises(DatasetError, match='strictly increasing'):
        read_imu(seq / 'mav0' / 'imu0')


def test_rejects_missing_images(tmp_path):
    seq = make_asl(tmp_path)
    (seq / 'mav0' / 'cam1' / 'data' / '200.png').unlink()
    with pytest.raises(DatasetError, match='missing'):
        read_camera_index(seq / 'mav0' / 'cam1')


def test_rejects_non_asl_folder(tmp_path):
    with pytest.raises(DatasetError):
        find_mav0(tmp_path)


def test_availability_full_coverage():
    stamps = [10.0 + 0.05 * k for k in range(201)]           # 10 s .. 20 s at 20 Hz
    a = availability(stamps, sensor_start_s=8.0, sensor_end_s=20.0)
    assert a.init_latency_s == pytest.approx(2.0)
    assert a.coverage == pytest.approx(1.0)
    assert a.output_rate_hz == pytest.approx(20.0)


def test_availability_penalizes_gaps_and_early_stop():
    stamps = [10.0 + 0.05 * k for k in range(41)]             # 10..12 s
    stamps += [14.0 + 0.05 * k for k in range(41)]            # 14..16 s, then silence to 20 s
    a = availability(stamps, sensor_start_s=10.0, sensor_end_s=20.0)
    assert a.coverage == pytest.approx(1.0 - (2.0 + 4.0) / 10.0)
    assert a.longest_gap_s == pytest.approx(4.0)


def test_availability_with_no_output():
    a = availability([], 0.0, 10.0)
    assert a.coverage == 0.0


def test_acceptance_reports_each_failure():
    result = {'ape_rmse_m': 0.5, 'matched_poses': 10,
              'availability': {'init_latency_s': 30.0, 'coverage': 0.5}}
    limits = {'max_ape_rmse_m': 0.3, 'max_init_latency_s': 15.0, 'min_coverage': 0.95,
              'min_matched_poses': 500}
    assert len(check_acceptance(result, limits)) == 4
    good = {'ape_rmse_m': 0.1, 'matched_poses': 1000,
            'availability': {'init_latency_s': 3.0, 'coverage': 0.99}}
    assert check_acceptance(good, limits) == []


def test_16bit_images_keep_the_high_byte(tmp_path):
    from PIL import Image as PILImage
    import numpy as np
    from ppb_vio_tools.euroc import CameraCalibration
    from ppb_vio_tools.images import load_image_mono8

    calib = CameraCalibration(4, 2, 1.0, 1.0, 1.0, 1.0, 'equidistant', (0, 0, 0, 0), None)
    msb = np.array([[0, 4, 256, 512], [32768, 40000, 65520, 65532]], dtype=np.uint16)
    PILImage.fromarray(msb).save(tmp_path / 'msb.png')
    pixels, ambiguous = load_image_mono8(tmp_path / 'msb.png', calib)
    assert pixels.tolist() == [0, 0, 1, 2, 128, 156, 255, 255] and not ambiguous

    PILImage.fromarray(np.full((2, 4), 200, dtype=np.uint16)).save(tmp_path / 'dark.png')
    assert load_image_mono8(tmp_path / 'dark.png', calib)[1]

    PILImage.fromarray(np.full((2, 4), 7, dtype=np.uint8)).save(tmp_path / 'mono8.png')
    pixels, ambiguous = load_image_mono8(tmp_path / 'mono8.png', calib)
    assert pixels.tolist() == [7] * 8 and not ambiguous

    PILImage.fromarray(np.zeros((3, 3), dtype=np.uint8)).save(tmp_path / 'wrong.png')
    with pytest.raises(DatasetError, match='size'):
        load_image_mono8(tmp_path / 'wrong.png', calib)


def test_input_health_flags_degraded_streams_inside_window_only():
    samples = [(t, 'ppb_vio: imu stream', 0) for t in range(10, 110)]
    samples += [(t, 'ppb_vio: camera 0 stream', 0) for t in range(10, 110)]
    samples += [(5, 'ppb_vio: imu stream', 3), (120, 'ppb_vio: imu stream', 2)]   # outside
    samples += [(50, 'ppb_vio: odometry', 2)]                                      # not an input
    fractions, degraded = input_health(samples, 10, 110)
    assert not degraded and fractions['ppb_vio: imu stream'] == 0.0

    samples += [(t + 0.5, 'ppb_vio: imu stream', 1) for t in range(60, 70)]       # 10 warnings
    fractions, degraded = input_health(samples, 10, 110)
    assert degraded and fractions['ppb_vio: imu stream'] > 0.02


def test_input_health_without_any_status_is_degraded():
    assert input_health([], 0, 10)[1]


def _lissajous(n=400):
    import numpy as np
    t = np.linspace(0, 20, n)
    return np.stack([3 * np.sin(0.7 * t), 2 * np.sin(1.1 * t), 0.5 * np.cos(0.3 * t)], axis=1)


def test_yaw_alignment_recovers_rotation_about_gravity():
    import numpy as np
    ref = _lissajous()
    yaw = 2.3
    rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    est = (ref - [1.0, -4.0, 0.3]) @ rz.T       # what an estimator in its own frame reports
    rot, t = align_position_yaw(ref, est)
    assert np.allclose(est @ rot.T + t, ref, atol=1e-9)
    assert np.allclose(position_errors_after_yaw_alignment(ref, est), 0.0, atol=1e-9)


def test_yaw_alignment_does_not_absorb_roll_or_pitch_error():
    import numpy as np
    ref = _lissajous()
    tilt = 0.05                                    # ~2.9 deg roll error in the estimate
    rx = np.array([[1, 0, 0], [0, np.cos(tilt), -np.sin(tilt)], [0, np.sin(tilt), np.cos(tilt)]])
    est = ref @ rx.T
    assert position_errors_after_yaw_alignment(ref, est).max() > 0.05


def test_yaw_alignment_rejects_bad_shapes():
    with pytest.raises(ValueError):
        align_position_yaw([[0, 0, 0]], [[0, 0, 0]])
    with pytest.raises(ValueError):
        align_position_yaw([[0, 0, 0], [1, 1, 1]], [[0, 0], [1, 1]])


def test_input_health_requires_reports_across_the_window():
    # Monitor stopped reporting after 30 of 100 s: input health is unknown.
    samples = [(t, 'ppb_vio: imu stream', 0) for t in range(10, 40)]
    samples += [(t, 'ppb_vio: camera 0 stream', 0) for t in range(10, 40)]
    assert input_health(samples, 10, 110)[1]
    # Camera status missing entirely.
    only_imu = [(t, 'ppb_vio: imu stream', 0) for t in range(10, 110)]
    assert input_health(only_imu, 10, 110)[1]
