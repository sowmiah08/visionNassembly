from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_path = get_package_share_directory('so101_description')

    xacro_file = os.path.join(
        pkg_path,
        'urdf',
        'dual_arm_workcell_gazebo.urdf.xacro'
    )

    world_file = os.path.join(
        pkg_path,
        'worlds',
        'test_world.sdf'
    )

    resource_path = os.pathsep.join([
        os.path.dirname(pkg_path),
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    ])

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    return LaunchDescription([

        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=resource_path,
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['so101_left_arm_controller'],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['so101_left_gripper_controller'],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['so101_right_arm_controller'],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['so101_right_gripper_controller'],
            output='screen'
        ),

        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen'
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'dual_arm_workcell',
                '-z', '0.0',
            ],
            output='screen'
        ),
    ])
