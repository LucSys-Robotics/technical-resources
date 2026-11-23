# Copyright 2026 LucSys Robotics - integration and maintenance changes only.
#
# LucSys Robotics distributes this source under supplier authorization.
# LucSys Robotics is responsible for integration and maintenance only.
# This notice does not grant an additional open-source license.
# Third-party components retain their respective licenses and notices.

from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    imu_processor = launch_ros.actions.Node(
        package='lucsys_chassis_driver',
        executable='ImuProcessor',
        name='imu_processor',
        # output='screen',
    )
    return LaunchDescription([
        imu_processor
    ])
