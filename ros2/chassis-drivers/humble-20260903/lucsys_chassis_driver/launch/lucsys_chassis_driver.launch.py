"""Start the base driver; explicitly opt into localization and model publishing."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Keep optional components independent from base serial bring-up."""
    directory = Path(get_package_share_directory('lucsys_chassis_driver')) / 'launch'
    actions = [
        DeclareLaunchArgument('start_ekf', default_value='false'),
        DeclareLaunchArgument('start_state_publisher', default_value='false'),
        DeclareLaunchArgument('publish_static_tf', default_value='false'),
    ]
    for filename, flag in [
        ('base_serial.launch.py', None),
        ('lucsys_ekf.launch.py', 'start_ekf'),
        ('robot_mode_description.launch.py', 'start_state_publisher'),
    ]:
        # The model entry also supports static-only operation through its own launch file.
        options = {} if flag is None else {'condition': IfCondition(LaunchConfiguration(flag))}
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(directory / filename)), **options))
    return LaunchDescription(actions)
