from pathlib import Path
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE = 'lucsys_lsr550_2wd_description'
MODELS = {'lsr550_2wd': 'LSR550-2WD'}

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
