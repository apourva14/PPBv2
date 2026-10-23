import json
from pathlib import Path

import pytest
import yaml

from ppb_vio.lib.estimator_config import apply_override, ConfigError, materialize, Override

CAMERA = """%YAML:1.0 # need to specify the file type at the top!

cam:
  enabled: true
  topic: "/not_this_one"

cam0:
  timeoffset: 0.0
  T_imu_cam:
    - [1.0, 0.0]
  topic: "/cam0/image_raw" # left
cam1:
  topic: "/cam1/image_raw"
"""

ESTIMATOR = """%YAML:1.0
verbosity: "INFO" # ALL, DEBUG
record_timing_information: false # timing
nested:
  record_timing_information: true
"""

PACKAGE_DIR = Path(__file__).resolve().parents[1]


def test_override_targets_only_the_named_section():
    out = apply_override(CAMERA, 'cam0', 'topic', '/vio/cam0/image_raw')
    assert 'topic: "/vio/cam0/image_raw" # left' in out
    assert 'topic: "/cam1/image_raw"' in out
    assert 'topic: "/not_this_one"' in out
    assert out.count('\n') == CAMERA.count('\n')


def test_top_level_override_ignores_nested_keys():
    out = apply_override(ESTIMATOR, '', 'record_timing_information', True)
    assert 'record_timing_information: true # timing' in out
    assert '  record_timing_information: true' in out


@pytest.mark.parametrize('section,key', [
    ('cam2', 'topic'),          # missing section
    ('cam0', 'rostopic'),       # missing key
])
def test_override_must_match_exactly_once(section, key):
    with pytest.raises(ConfigError):
        apply_override(CAMERA, section, key, 'x')


def test_override_rejects_duplicate_keys():
    text = 'imu:\n  topic: "/a"\n  topic: "/b"\n'
    with pytest.raises(ConfigError):
        apply_override(text, 'imu', 'topic', '/vio/imu')


def test_values_are_formatted_for_opencv_yaml():
    text = 'sys:\n  a: 1\n  b: x\n  c: 0.5\n'
    out = apply_override(text, 'sys', 'a', False)
    out = apply_override(out, 'sys', 'b', '/path/file.txt')
    out = apply_override(out, 'sys', 'c', 2.5)
    assert out == 'sys:\n  a: false\n  b: "/path/file.txt"\n  c: 2.5\n'


def test_materialize_copies_edits_and_writes_manifest(tmp_path):
    src = tmp_path / 'upstream'
    src.mkdir()
    (src / 'config_camera.yaml').write_text(CAMERA)
    (src / 'config.yaml').write_text('%YAML:1.0\n')
    manifest = materialize(src, tmp_path / 'run', [
        Override('config_camera.yaml', 'cam0', 'topic', '/vio/cam0/image_raw')])
    assert '/vio/cam0/image_raw' in (tmp_path / 'run' / 'config_camera.yaml').read_text()
    assert (src / 'config_camera.yaml').read_text() == CAMERA          # source untouched
    assert json.loads(manifest.read_text())['overrides'][0]['key'] == 'topic'


def test_materialize_refuses_extra_sensor_configs(tmp_path):
    src = tmp_path / 'upstream'
    src.mkdir()
    (src / 'config_wheel.yaml').write_text('wheel:\n  enabled: true\n')
    with pytest.raises(ConfigError, match='extra sensors'):
        materialize(src, tmp_path / 'run', [], forbidden_files=['config_wheel.yaml'])


def test_materialize_refuses_to_overwrite(tmp_path):
    src = tmp_path / 'upstream'
    src.mkdir()
    (tmp_path / 'run').mkdir()
    with pytest.raises(ConfigError, match='overwrite'):
        materialize(src, tmp_path / 'run', [])


@pytest.mark.parametrize('profile', sorted((PACKAGE_DIR / 'config' / 'estimators').glob('*.yaml')))
def test_shipped_profiles_are_well_formed(profile):
    p = yaml.safe_load(profile.read_text())
    assert p['estimator'] in ('mins', 'openvins')
    assert p['output']['linear_velocity_frame'] in ('child', 'parent')
    assert p['output']['odom_topic'] not in ('/robot/odom', '/vio/odom')
    for o in p['overrides']:
        Override(o['file'], o['section'], o['key'], o['value'])
    topics = [o['value'] for o in p['overrides'] if o['key'] in ('topic', 'rostopic')]
    assert topics and all(t.startswith('/vio/') for t in topics)
    if p['estimator'] == 'mins':
        # Camera + IMU only: every other MINS sensor config must be forbidden.
        assert set(p['forbidden_files']) == {
            'config_wheel.yaml', 'config_gps.yaml', 'config_lidar.yaml', 'config_vicon.yaml'}
