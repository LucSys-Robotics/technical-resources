# Copyright 2026 LucSys Robotics - integration and maintenance changes only.
#
# LucSys Robotics distributes this source under supplier authorization.
# LucSys Robotics is responsible for integration and maintenance only.
# This notice does not grant an additional open-source license.
# Third-party components retain their respective licenses and notices.

"""Publish an explicitly selected LucSys model; never infer configuration compatibility."""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def spawn_robot_nodes(context):
    """Load the requested installed model and optional profile transforms."""
    def value(key): return LaunchConfiguration(key).perform(context)
    actions = []
    if value('start_state_publisher').lower() == 'true':
        package, model = value('model_package'), value('model_name')
        if not package.startswith('lucsys_') or not package.endswith('_description'):
            raise ValueError(
                'Set model_package to the installed LucSys model package '
                'for the exact configuration.')
        if not model or Path(model).name != model or model in ('.', '..'):
            raise ValueError('Set model_name to an existing URDF basename in that package.')
        path = Path(get_package_share_directory(package)) / 'urdf' / (model + '.urdf')
        description = path.read_text(encoding='utf-8')
        actions.extend([
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 parameters=[{'robot_description': ParameterValue(description, value_type=str)}]),
            Node(package='joint_state_publisher', executable='joint_state_publisher'),
        ])
    if value('publish_static_tf').lower() == 'true':
        share = Path(get_package_share_directory('lucsys_chassis_driver'))
        cfg = yaml.safe_load(Path(value('lucsys_param_yaml')).read_text())
        aliases = yaml.safe_load(
            (share / 'config/legacy_compatibility.yaml').read_text())['model_aliases']
        original_mode = value('car_mode') or cfg['car_mode']
        mode = aliases.get(original_mode, original_mode)
        profiles = yaml.safe_load(Path(value('robot_model_yaml')).read_text())['robot_model']
        if mode not in profiles and original_mode in profiles:
            mode = original_mode
        if mode not in profiles:
            raise ValueError('No transform profile for car_mode: ' + mode)
        for key, child in {'base_to_laser': 'laser', 'base_to_camera': 'camera_link',
                           'base_to_link': 'base_link', 'base_to_gyro': 'gyro_link',
                           'base_to_radar': 'radar'}.items():
            if key not in profiles[mode]:
                continue
            data = profiles[mode][key]
            arguments = []
            for axis, number in zip(('x', 'y', 'z', 'yaw', 'pitch', 'roll'), data):
                arguments.extend(['--' + axis, str(number)])
            actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
                                name=key, arguments=arguments + [
                                    '--frame-id', 'base_footprint', '--child-frame-id', child]))
    return actions


def generate_launch_description():
    """Require an explicit model package; leave unverified transforms disabled."""
    share = Path(get_package_share_directory('lucsys_chassis_driver'))
    args = {
        'model_package': '', 'model_name': '', 'start_state_publisher': 'true',
        'publish_static_tf': 'false', 'car_mode': '',
        'lucsys_param_yaml': str(share / 'config/lucsys_param.yaml'),
        'robot_model_yaml': str(share / 'config/robot_model.yaml'),
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=value) for key, value in args.items()],
        OpaqueFunction(function=spawn_robot_nodes),
    ])
