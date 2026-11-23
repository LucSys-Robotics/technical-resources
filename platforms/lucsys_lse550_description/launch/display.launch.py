from pathlib import Path
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE = 'lucsys_lse550_description'
MODELS = {'lse550_4wd_four_axis_arm': 'LSE550-4WD', 'lse550_4wd': 'LSE550-4WD', 'lse550_akm': 'LSE550-AKM', 'lse550_diff': 'LSE550-DIFF', 'lse550_mec_four_axis_arm': 'LSE550-MEC', 'lse550_mec_six_axis_arm': 'LSE550-MEC', 'lse550_mec': 'LSE550-MEC', 'lse550_omni': 'LSE550-OMNI', 'lse550_tank_four_axis_arm': 'LSE550-TANK', 'lse550_tank': 'LSE550-TANK'}

def _launch(context):
    share = Path(get_package_share_directory(PACKAGE))
    model = LaunchConfiguration('model').perform(context)
    if model not in MODELS:
        raise ValueError('Unknown model: ' + model)
    description = (share / 'urdf' / (model + '.urdf')).read_text(encoding='utf-8')
    root = ET.fromstring(description)
    for mesh in root.findall('.//mesh'):
        uri = mesh.attrib['filename']
        prefix = 'package://' + PACKAGE + '/'
        if not uri.startswith(prefix) or not (share / uri[len(prefix):]).is_file():
            raise FileNotFoundError('Required model mesh is unavailable: ' + uri)
    parameters = {'robot_description': ParameterValue(description, value_type=str),
                  'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen', parameters=[parameters]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             name='joint_state_publisher_gui', parameters=[parameters],
             condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             name='joint_state_publisher', parameters=[parameters],
             condition=UnlessCondition(LaunchConfiguration('gui'))),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', str(share / 'rviz' / (model + '.rviz'))],
             parameters=[{'use_sim_time': parameters['use_sim_time']}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=next(iter(MODELS)), choices=list(MODELS),
                              description='LucSys Robotics model configuration'),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=_launch),
    ])
