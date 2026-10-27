"""Write the estimator config for one offline bench run (same as vio.launch.py does live).

    prepare_estimator_config --profile openvins_rosario_v1 --run-dir /data/runs/<run> \
        [--calibration-dir /data/rosario_v1/sequence02/openvins]

Writes <run-dir>/estimator_config/ and <run-dir>/profile.yaml, prints the entry config path.
"""

import argparse
from pathlib import Path
import shutil
import sys

from ament_index_python.packages import get_package_share_directory
import yaml

from ppb_vio.lib.estimator_config import ConfigError, materialize_profile


def resolve(kind, value):
    path = Path(value)
    if path.suffix == '.yaml' and path.is_file():
        return path
    candidate = Path(get_package_share_directory('ppb_vio')) / 'config' / kind / f'{value}.yaml'
    if not candidate.is_file():
        raise FileNotFoundError(f'{kind} profile not found: {value}')
    return candidate


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--profile', required=True, help='estimator profile name or path')
    parser.add_argument('--run-dir', required=True)
    parser.add_argument('--calibration-dir', help='generated calibration files, if the profile needs them')
    args = parser.parse_args(argv)

    profile_path = resolve('estimators', args.profile)
    profile = yaml.safe_load(profile_path.read_text())
    run_dir = Path(args.run_dir).resolve()
    run_dir.mkdir(parents=True, exist_ok=True)
    upstream = profile['upstream']
    src_dir = Path(get_package_share_directory(upstream['package'])) / upstream['config_dir']
    try:
        entry = materialize_profile(profile, src_dir, run_dir, args.calibration_dir)
    except ConfigError as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 1
    shutil.copyfile(profile_path, run_dir / 'profile.yaml')
    print(entry)
    return 0


if __name__ == '__main__':
    sys.exit(main())
