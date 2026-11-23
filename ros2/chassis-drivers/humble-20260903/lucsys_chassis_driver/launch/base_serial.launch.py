"""Launch the chassis driver with explicit hardware settings."""
from pathlib import Path
import warnings

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_nodes(context):
    """Resolve the selected profile without assuming hardware compatibility."""
    share = Path(get_package_share_directory('lucsys_chassis_driver'))
    cfg = yaml.safe_load(Path(LaunchConfiguration('lucsys_param_yaml').perform(context)).read_text())
    compat = yaml.safe_load((share / 'config/legacy_compatibility.yaml').read_text())
    imu_path = LaunchConfiguration('imu_mode_yaml').perform(context)
    imu_cfg = yaml.safe_load(Path(imu_path).read_text()) if imu_path else cfg
    imu = LaunchConfiguration('imu_mode').perform(context) or imu_cfg['imu_mode']
    mode = LaunchConfiguration('car_mode').perform(context) or cfg['car_mode']
    mode = compat['model_aliases'].get(mode, mode)
    port = LaunchConfiguration('usart_port_name').perform(context)
    legacy = LaunchConfiguration('legacy_compatibility').perform(context).lower() == 'true'
    if legacy and port == '/dev/lucsys_controller' and not Path(port).exists():
        if Path(compat['controller_device']).exists():
            port = compat['controller_device']
            warnings.warn('Using a legacy serial-device alias; migrate to /dev/lucsys_controller.')
    params = {
        'usart_port_name': port,
        'serial_baud_rate': int(LaunchConfiguration('serial_baud_rate').perform(context)),
        'robot_frame_id': 'base_footprint', 'odom_frame_id': 'odom_combined',
        'cmd_vel': 'cmd_vel', 'akm_cmd_vel': 'none', 'product_number': 0,
        'odom_x_scale': 1.0, 'odom_y_scale': 1.0,
        'odom_z_scale_positive': 1.0, 'odom_z_scale_negative': 1.0,
        'car_mode': mode, 'ultrasonic_avoid': cfg['ultrasonic_avoid'],
    }
    actions = []
    if cfg['ultrasonic_avoid']:
        package = LaunchConfiguration('ultrasonic_package').perform(context)
        if not package and legacy:
            package = compat['ultrasonic_package']
        if not package:
            raise ValueError('Ultrasonic avoidance is enabled: specify its installed package.')
        sensor = Path(get_package_share_directory(package)) / 'launch/ultrasonic_pose.launch.py'
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(sensor))))
    if imu not in ('stm32', 'H30', 'N300pro'):
        raise ValueError('Unsupported imu_mode: ' + imu)
    actions.append(Node(
        package='lucsys_chassis_driver', executable='lucsys_chassis_node',
        name=LaunchConfiguration('node_name').perform(context),
        output='screen', parameters=[params],
        remappings=[] if imu == 'stm32' else [('imu/data_raw', 'imu/data_board')],
    ))
    if imu != 'stm32':
        package, filename = {
            'H30': ('yesense_std_ros2', 'yesense_node.launch.py'),
            'N300pro': ('hipnuc_imu', 'imu_spec_msg.launch.py'),
        }[imu]
        sensor = Path(get_package_share_directory(package)) / 'launch' / filename
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(sensor))))
    return actions


def generate_launch_description():
    """Declare serial and configuration arguments."""
    default = str(Path(get_package_share_directory('lucsys_chassis_driver')) / 'config/lucsys_param.yaml')
    args = {
        'lucsys_param_yaml': default, 'imu_mode_yaml': '', 'imu_mode': '', 'car_mode': '',
        'usart_port_name': '/dev/lucsys_controller', 'serial_baud_rate': '115200',
        'legacy_compatibility': 'true', 'ultrasonic_package': '', 'node_name': 'lucsys_robot',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=value) for key, value in args.items()],
        OpaqueFunction(function=launch_nodes),
    ])
