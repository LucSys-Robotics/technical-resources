"""Start the base driver with explicitly enabled optional sensors."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Do not require optional sensor SDKs for base-driver startup."""
    directory = Path(get_package_share_directory('lucsys_chassis_driver')) / 'launch'
    return LaunchDescription([
        DeclareLaunchArgument('start_lidar', default_value='false'),
        DeclareLaunchArgument('start_camera', default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(directory / 'lucsys_chassis_driver.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(directory / 'lucsys_lidar.launch.py')),
                                 condition=IfCondition(LaunchConfiguration('start_lidar'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(directory / 'lucsys_camera.launch.py')),
                                 condition=IfCondition(LaunchConfiguration('start_camera'))),
    ])
