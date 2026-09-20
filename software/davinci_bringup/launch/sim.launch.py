"""Startup wiring only; robot kinematics, motion, and verification are C++."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def start(context):
    dof = LaunchConfiguration('dof').perform(context)
    backend = LaunchConfiguration('backend').perform(context)
    gui = LaunchConfiguration('gui').perform(context) == 'true'
    if dof not in ('3', '4') or backend not in ('gazebo', 'mock'):
        raise ValueError('Use dof:=3|4 and backend:=gazebo|mock')
    description_share = Path(get_package_share_directory('davinci_description'))
    control_share = Path(get_package_share_directory('davinci_control'))
    controllers = str(control_share / 'config' / f'controllers_{dof}dof.yaml')
    robot = xacro.process_file(
        str(description_share / 'urdf' / 'arm.urdf.xacro'),
        mappings={'dof': dof, 'backend': backend, 'controllers_file': controllers},
    ).toxml()
    actions = [
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            parameters=[{'robot_description': robot, 'use_sim_time': backend == 'gazebo'}],
            output='screen',
        ),
    ]
    if backend == 'gazebo':
        gz_share = Path(get_package_share_directory('ros_gz_sim'))
        lab_share = Path(get_package_share_directory('davinci_gazebo'))
        world = lab_share / 'worlds' / 'one_arm_lab.sdf'
        actions.extend([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(gz_share / 'launch' / 'gz_sim.launch.py')),
                launch_arguments={
                    'gz_args': f'{"-s " if not gui else ""}-r "{world}"',
                    'on_exit_shutdown': 'true',
                }.items(),
            ),
            Node(
                package='ros_gz_bridge', executable='parameter_bridge',
                arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
                output='screen',
            ),
            Node(
                package='ros_gz_sim', executable='create',
                arguments=['-name', 'davinci_arm', '-topic', '/robot_description',
                           '-allow_renaming', 'false'],
                output='screen',
            ),
        ])
    else:
        actions.append(Node(
            package='controller_manager', executable='ros2_control_node',
            parameters=[controllers, {'use_sim_time': False}],
            output='screen',
        ))
    actions.append(Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', 'arm_controller',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '120'],
        output='screen',
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dof', default_value='3', choices=['3', '4']),
        DeclareLaunchArgument('backend', default_value='gazebo', choices=['gazebo', 'mock']),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=start),
    ])
