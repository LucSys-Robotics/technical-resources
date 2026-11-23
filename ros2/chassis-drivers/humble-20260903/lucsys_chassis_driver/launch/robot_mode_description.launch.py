import os
import yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

def load_yaml(file_path: Path) -> dict:
    """Load the YAML configuration file"""
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)


def spawn_robot_nodes(context, *args, **kwargs):
    """
    Functions that truly construct nodes, called by OpaqueFunction
    Decide whether to start the robot_state_publisher based on the parameters
    """
    # ========== 1. Read configuration file ==========
    param_file = LaunchConfiguration('lucsys_param_yaml').perform(context)
    robot_model_file = LaunchConfiguration('robot_model_yaml').perform(context)
    cfg = load_yaml(Path(param_file))
    model_cfg = load_yaml(Path(robot_model_file))

    # ========== 2. Get the model configuration ==========
    car_mode = LaunchConfiguration('car_mode').perform(context) or cfg['car_mode']
    print(f'[robot_mode_description] 车型: {car_mode}')

    if car_mode not in model_cfg['robot_model']:
        raise ValueError(f'未知车型 "{car_mode}"，请检查 {param_file} 和 {robot_model_file}')

    model_cfg = model_cfg['robot_model'][car_mode]

    # ========== 3. Get robot_state_publisher startup parameter ==========
    start_state_publisher = LaunchConfiguration('start_state_publisher').perform(context) == 'true'
    print(f'[robot_mode_description] 启动robot_state_publisher: {start_state_publisher}')

    # ========== 4. Construct the URDF file path ==========
    urdf_path = os.path.join(
        get_package_share_directory('wheeltec_robot_urdf'),
        'urdf',
        f'{car_mode}_robot.urdf'
    )

    # ========== 5. Construct node list ==========
    actions = []

    # --- 5.1 robot_state_publisher (Conditional Starting) ---
    if start_state_publisher:
        if os.path.exists(urdf_path):
            actions.append(
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    parameters=[{
                        'robot_description': ParameterValue(
                            Command(['xacro ', urdf_path]),
                            value_type=str
                        )
                    }],
                ),
            )
            actions.append(
                Node(
                    package='joint_state_publisher',
                    executable='joint_state_publisher',
                    name='joint_state_publisher',
                )
            )
        else:
            print(f'[robot_mode_description] URDF文件不存在')
            
    # --- 5.2 Static TF Released---
    # Static TF configuration: { 'YAML configuration key (also node name)' : 'child_frame_id' }
    static_transforms = {
        'base_to_laser':  'laser',
        'base_to_camera': 'camera_link',
        'base_to_link':   'base_link',
        'base_to_gyro':   'gyro_link',
        'base_to_radar':  'radar',
    }
    parent_frame = 'base_footprint'
    # Batch generate nodes using the dictionary items().
    for cfg_key, child_frame in static_transforms.items():
        if cfg_key in model_cfg:
            tf_data = model_cfg[cfg_key]
            actions.append(
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name=cfg_key,  # Directly reuse cfg_key as the node name to eliminate redundancy
                    arguments=[
                        '--x', str(tf_data[0]),
                        '--y', str(tf_data[1]),
                        '--z', str(tf_data[2]),
                        '--yaw', str(tf_data[3]),
                        '--pitch', str(tf_data[4]),
                        '--roll', str(tf_data[5]),
                        '--frame-id', parent_frame,
                        '--child-frame-id', child_frame,
                    ],
                )
            )
        else:
            print(f'[robot_mode_description] 警告: {robot_model_file} 中缺少 {cfg_key} 配置，已跳过 {child_frame} 的 TF 发布')
    return actions


# ========== Launch description ==========
def generate_launch_description():
    """
    Generate launch description
    Supported parameters:
    - lucsys_param_yaml: Chassis parameter profile path
    - robot_model_yaml: Path to the robot model profile
    - car_mode: Select the vehicle model (optional, default uses the configuration in YAML)
    - start_state_publisher: Whether to start robot_state_publisher (default true)
    """
    return LaunchDescription([
        # ========== Parameter declaration ==========

        # Parameter 1: Chassis profile path
        DeclareLaunchArgument(
            'lucsys_param_yaml',
            default_value=os.path.join(
                get_package_share_directory('lucsys_chassis_driver'),
                'config', 'lucsys_param.yaml'
            ),
            description='lucsys_param.yaml配置文件路径'
        ),

        # Parameter 2: Path to the robot model profile
        DeclareLaunchArgument(
            'robot_model_yaml',
            default_value=os.path.join(
                get_package_share_directory('lucsys_chassis_driver'),
                'config', 'robot_model.yaml'
            ),
            description='robot_model.yaml配置文件路径'
        ),

        # Parameter 3: Vehicle Selection (Optional)
        DeclareLaunchArgument(
            'car_mode',
            default_value='',   # An empty string means using the default value in YAML
            description='车型选择 (mini_akm, mini_mec, S300等)'
        ),
 
        # Parameter 4: Whether to start the robot_state_publisher, joint_state_publisher
        DeclareLaunchArgument(
            'start_state_publisher',
            default_value='true',
            description='是否启动robot_state_publisher,joint_state_publisher（默认true；与机械臂联用时设为false）'
        ),

        # ========== Use OpaqueFunction to parse yaml at runtime and construct node ==========
        OpaqueFunction(function=spawn_robot_nodes)
    ])
