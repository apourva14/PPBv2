"""Local fusion: vio_gate + robot_localization ekf_local (odom -> base_link).

    ros2 launch ppbv2_localization local_ekf.launch.py [variants:=vio_twist,imu_gyro_only]
        [use_sim_time:=true]

Inputs (spec, "Topic/frame contracts"): /imu/data, /wheel/odom, /vio/odom, /vio/status,
/tf_static (base_link -> imu_link). Output: /odometry/local and TF odom -> base_link.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'ppbv2_localization'


def _setup(context):
    share = Path(get_package_share_directory(PKG)) / 'config'
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    params = [str(share / 'ekf_local.yaml')]
    for name in filter(None, LaunchConfiguration('variants').perform(context).split(',')):
        path = share / 'variants' / f'{name.strip()}.yaml'
        if not path.is_file():
            raise FileNotFoundError(f'unknown variant {name!r} ({path})')
        params.append(str(path))
    return [
        Node(package=PKG, executable='vio_gate', name='vio_gate', output='screen',
             parameters=[{'use_sim_time': use_sim_time}]),
        Node(package='robot_localization', executable='ekf_node', name='ekf_local',
             output='screen', parameters=params + [{'use_sim_time': use_sim_time}],
             remappings=[('odometry/filtered', '/odometry/local')]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('variants', default_value='',
                              description='comma-separated config/variants/*.yaml to layer on top'),
        OpaqueFunction(function=_setup),
    ])
