# Copyright 2026 LucSys Robotics - integration and maintenance changes only.
#
# LucSys Robotics distributes this source under supplier authorization.
# LucSys Robotics is responsible for integration and maintenance only.
# This notice does not grant an additional open-source license.
# Third-party components retain their respective licenses and notices.

import os
import yaml
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    GroupAction, OpaqueFunction
)
from launch_ros.actions import LifecycleNode
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def load_yaml(path: Path) -> dict:
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def include_lidar_launch(context, *args, **kwargs):
    # 1. Read parameter file
    yaml_path = LaunchConfiguration('lidar_type_yaml').perform(context)
    cfg = load_yaml(Path(yaml_path))

    # 2. Get lidar_type (command line priority> yaml default)
    lidar_type = LaunchConfiguration('lidar_type').perform(context) or cfg['lidar_type']
    print(f'lidar_type:{lidar_type}')
    # 3. Decide the package path based on the model
    actions = []
    if lidar_type.startswith('ls'):            # LS series
        launch_dir = os.path.join(get_package_share_directory('lslidar_driver'), 'launch')
        # Detects whether the radar type ends with _double
        if lidar_type.endswith('_double'):
            print('[LucSys LiDAR] Dual-LiDAR mode: starting both drivers and fusion.')
            lidar_launch = GroupAction(
                actions=[
                    # 1. Start dual radar driver
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(launch_dir, 'lslidar_m10p_double.launch.py')
                        ),
                    ),
                    # 2. Initiated dual-radar fusion
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(
                                get_package_share_directory('double_lidar_fusion'),
                                'launch',
                                'double_lidar_fusion.launch.py'
                            )
                        ),
                    ),
                ]
            )
        elif lidar_type == 'lscx':
            template_yaml = Path(
                get_package_share_directory('lslidar_driver'),
                'config', 'lslidar_cx.yaml'
            )
            cx_cfg = yaml.safe_load(template_yaml.read_text())[
                'cx']['lslidar_driver_node']['ros__parameters']
            if cfg['lscx']['angle_disable_min'] != 0 and cfg['lscx']['angle_disable_max'] != 0:
                cx_cfg['angle_disable_min'] = cfg['lscx']['angle_disable_min']
                cx_cfg['angle_disable_max'] = cfg['lscx']['angle_disable_max']

            lidar_launch = GroupAction(
                actions=[
                    LifecycleNode(package='lslidar_driver',
                                  executable='lslidar_driver_node',
                                  name='lslidar_driver_node',
                                  # Matches the namespace in the corresponding YAML file
                                  namespace='cx',
                                  parameters=[cx_cfg],
                                  output='screen'
                                  ),
                    IncludeLaunchDescription(PythonLaunchDescriptionSource(
                        os.path.join(get_package_share_directory('pointcloud_to_laserscan'),
                                     'launch', 'pointcloud_to_laserscan_launch.py')),
                    )
                ]
            )
        else:
            template_yaml = Path(
                get_package_share_directory('lslidar_driver'),
                'config', 'lslidar_x10.yaml'
            )
            lidar_port = cfg['x10']['lidar_port']
            compat_path = Path(get_package_share_directory(
                'lucsys_chassis_driver')) / 'config/legacy_compatibility.yaml'
            legacy_port = load_yaml(compat_path)['lidar_device']
            if lidar_port == '/dev/lucsys_lidar' and not Path(
                    lidar_port).exists() and Path(legacy_port).exists():
                lidar_port = legacy_port
                print('Using legacy LiDAR device alias; migrate to /dev/lucsys_lidar.')
            x10_cfg = yaml.safe_load(template_yaml.read_text())[
                'x10']['lslidar_driver_node']['ros__parameters']

            if lidar_type.endswith('net'):
                x10_cfg['serial_port'] = ''
            elif lidar_type.endswith('uart'):
                x10_cfg['serial_port'] = lidar_port
            if lidar_type.startswith('ls_M10'):
                if lidar_type.startswith('ls_M10P'):
                    x10_cfg['lidar_model'] = 'M10P'
                else:
                    x10_cfg['lidar_model'] = 'M10'
            if lidar_type.startswith('ls_N10'):
                if lidar_type.startswith('ls_N10Plus'):
                    x10_cfg['lidar_model'] = 'N10Plus'
                else:
                    x10_cfg['lidar_model'] = 'N10'

            if cfg['x10']['angle_disable_min'] != 0 and cfg['x10']['angle_disable_max'] != 0:
                x10_cfg['angle_disable_min'] = cfg['x10']['angle_disable_min']
                x10_cfg['angle_disable_max'] = cfg['x10']['angle_disable_max']

            lidar_launch = LifecycleNode(package='lslidar_driver',
                                         executable='lslidar_driver_node',
                                         name='lslidar_driver_node',
                                         # Matches the namespace in the corresponding YAML file
                                         namespace='x10',
                                         parameters=[x10_cfg],
                                         output='screen'
                                         )

    elif lidar_type == 'ldstl19p':
        lidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ldlidar'),
                             'launch', 'stl19p.launch.py')))
    elif lidar_type == 'ldstl06nbj':
        lidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ldlidar'),
                             'launch', 'stl06nbj.launch.py')))
    elif lidar_type == 'ldstl19n':
        lidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ldlidar'),
                             'launch', 'stl19n.launch.py')))
    elif lidar_type == 'rplidar_c1':
        lidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('rplidar_ros'),
                             'launch', 'rplidar_c1_launch.py')))
    else:
        raise ValueError(f'Unsupported lidar: {lidar_type}')
    # 4. Return to the action list
    actions.append(lidar_launch)
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar_type_yaml',
            default_value=os.path.join(
                get_package_share_directory('lucsys_chassis_driver'),
                'config', 'lucsys_param.yaml'),
            description='Path to lidar_type.yaml'),
        DeclareLaunchArgument(
            'lidar_type',
            default_value='',
            description='Which lidar model to launch'),
        OpaqueFunction(function=include_lidar_launch),
    ])
