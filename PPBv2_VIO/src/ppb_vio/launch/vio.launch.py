"""Run one VIO estimator on the /vio/* sensor topics and publish /vio/odom.

    ros2 launch ppb_vio vio.launch.py estimator:=mins_euroc platform:=euroc \
        run_dir:=/data/runs/<name>

The workspace of the chosen estimator must be sourced (scripts/vio_env.sh).
Nothing here publishes /robot/odom or velocity commands.

Per run, <run_dir> receives:
    estimator_config/  upstream config + PPBv2 overrides actually used
    estimator/         estimator's own trajectory / timing files
    record/            ROS 2 bag of /vio/odom, /vio/status, estimator output, TF
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

from ppb_vio.lib.estimator_config import materialize, Override

PKG = 'ppb_vio'


def _bool(context, name):
    value = LaunchConfiguration(name).perform(context).lower()
    if value not in ('true', 'false'):
        raise ValueError(f'launch argument {name} must be true or false, got {value!r}')
    return value == 'true'


def _resolve(kind, value):
    """A profile is either a path to a YAML file or a name under config/<kind>/."""
    path = Path(value)
    if path.suffix == '.yaml' and path.is_file():
        return path
    candidate = Path(get_package_share_directory(PKG)) / 'config' / kind / f'{value}.yaml'
    if not candidate.is_file():
        raise FileNotFoundError(f'{kind} profile not found: {value}')
    return candidate


def _estimator_node(profile, entry_config, use_sim_time):
    name = profile['estimator']
    if name == 'mins':
        return Node(package='mins', executable='subscribe', name='mins_subscribe',
                    output='screen',
                    parameters=[{'config_path': entry_config, 'use_sim_time': use_sim_time}])
    if name == 'openvins':
        return Node(package='ov_msckf', executable='run_subscribe_msckf', namespace='ov_msckf',
                    name='run_subscribe_msckf', output='screen',
                    parameters=[{'config_path': entry_config, 'use_sim_time': use_sim_time}])
    raise ValueError(f'unknown estimator {name!r}')


def _setup(context):
    run_dir = Path(LaunchConfiguration('run_dir').perform(context)).resolve()
    use_sim_time = _bool(context, 'use_sim_time')
    profile_path = _resolve('estimators', LaunchConfiguration('estimator').perform(context))
    platform_path = _resolve('platforms', LaunchConfiguration('platform').perform(context))
    profile = yaml.safe_load(profile_path.read_text())

    if run_dir.exists() and any(run_dir.iterdir()):
        raise FileExistsError(f'run_dir {run_dir} is not empty; use a new directory per run')
    run_dir.mkdir(parents=True, exist_ok=True)
    upstream = profile['upstream']
    src_dir = Path(get_package_share_directory(upstream['package'])) / upstream['config_dir']
    overrides = [
        Override(o['file'], o['section'], o['key'],
                 o['value'].format(run_dir=run_dir) if isinstance(o['value'], str) else o['value'])
        for o in profile['overrides']
    ]
    cfg_dir = run_dir / 'estimator_config'
    materialize(src_dir, cfg_dir, overrides, profile.get('forbidden_files', []))
    entry_config = str(cfg_dir / upstream['entry_file'])

    estimator = _estimator_node(profile, entry_config, use_sim_time)
    adapter = Node(
        package=PKG, executable='vio_odom_adapter', name='vio_odom_adapter', output='screen',
        parameters=[str(platform_path), {
            'input_topic': profile['output']['odom_topic'],
            'input_linear_velocity_frame': profile['output']['linear_velocity_frame'],
            'publish_tf': _bool(context, 'publish_tf'),
            'use_sim_time': use_sim_time,
        }])
    monitor = Node(
        package=PKG, executable='vio_stream_monitor', name='vio_stream_monitor', output='screen',
        parameters=[str(platform_path), {'use_sim_time': use_sim_time}])

    actions = [estimator, adapter, monitor]
    # A dead estimator or adapter must end the run, never leave it half-alive.
    for label, proc in (('estimator', estimator), ('vio_odom_adapter', adapter)):
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=proc,
            on_exit=[EmitEvent(event=Shutdown(reason=f'{label} exited'))])))

    if _bool(context, 'record'):
        record_dir = run_dir / 'record'
        topics = ['/vio/odom', '/vio/status', profile['output']['odom_topic'],
                  '/tf', '/tf_static']
        actions.append(ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '--storage', 'mcap', '--output', str(record_dir)]
            + topics,
            output='screen'))

    actions.append(Node(
        package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge',
        condition=IfCondition(LaunchConfiguration('foxglove')),
        parameters=[{'port': 8765, 'address': '0.0.0.0', 'use_sim_time': use_sim_time}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('estimator', description='profile name or path, e.g. mins_euroc'),
        DeclareLaunchArgument('platform', description='platform name or path, e.g. euroc'),
        DeclareLaunchArgument('run_dir', description='new output directory for this run'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='true for bag replay (bag must publish /clock)'),
        # Off by default: on the robot base_link must not gain a vio_world parent.
        DeclareLaunchArgument('publish_tf', default_value='false',
                              description='broadcast vio_world -> base_link on /tf (replay only)'),
        DeclareLaunchArgument('record', default_value='true'),
        DeclareLaunchArgument('foxglove', default_value='false'),
        OpaqueFunction(function=_setup),
    ])
