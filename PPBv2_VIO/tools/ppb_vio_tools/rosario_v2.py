"""The Rosario Dataset v2 (Soncini et al., 2025): STUB, not implemented.

Deferred until the fusion phase. Recorded here so the loader can be written
without re-surveying (checked 2026-10-22 against sequence 2023-12-22-13-14-16):

- Data: http://fs01.cifasis-conicet.gov.ar:90/~robot_desmalezador/rosariov2/sequences/
  <id>.compressed.bag is 29-92 GB (ROS 1); small add-ons per sequence:
  _odom_fix.bag (/odom_fix, corrected wheel odometry), _conventional_gnss.bag
  (/reach_{1,2,3}/gps/fix: single-point, ~5 m from PPK), _ppk_gnss.bag
  (/reach_{1,2,3}/ppk/fix), _pgt.bag (ground truth, 3-28 GB).
- Ground truth as TUM CSV (RealSense IMU frame, ENU at its first pose):
  sequences_plain/mins_tum/<id>_mins_tum.csv. It is a MINS fusion of stereo,
  IMU and the same PPK GNSS, so it is not independent of the GNSS inputs.
- Tooling (msg definitions, URDF extrinsics): github.com/CIFASIS/rosariov2 @ 82115db620b5.
- reach_1 and reach_2 are 0.80 m apart across the robot (PPK baseline 0.799 +- 0.005 m):
  usable to synthesize a dual-antenna heading.
"""

from ppb_vio_tools.euroc import DatasetError


def convert(*_args, **_kwargs):
    raise DatasetError('Rosario v2 loader is a stub (deferred to the fusion phase); '
                       'see the module docstring')
