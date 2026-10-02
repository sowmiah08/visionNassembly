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
        'dual_arm_workcell_peghole_gazebo.urdf.xacro'
    )

    world_file = os.path.join(
        pkg_path,
        'worlds',
        'test_world.sdf'
    )

    rviz_config_file = os.path.join(
        pkg_path,
        'rviz',
        'workcell_cameras.rviz'
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
                '-name', 'dual_arm_workcell_peghole',
                '-z', '0.0',
            ],
            output='screen'
        ),

        # Bridge every camera's images from Gazebo transport to ROS 2 so
        # they can be viewed (rqt_image_view) or recorded (ros2 bag record).
        # image_bridge also republishes each as compressed/theora/zstd
        # (image_transport) variants automatically.
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=[
                '/left_wrist_camera/image_raw',
                '/right_wrist_camera/image_raw',
                '/overhead_camera/rgb/image_raw',
                '/overhead_camera/depth/image_raw',
            ],
            output='screen'
        ),

        # image_bridge does not also bridge camera_info, so that goes through
        # the generic parameter_bridge with explicit ROS <-> Gazebo type
        # mappings. (The depth point cloud is intentionally not bridged --
        # not needed, and heavier than the plain depth image.)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/left_wrist_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/right_wrist_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/overhead_camera/rgb/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/overhead_camera/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                # robot_state_publisher and rviz2 both run with
                # use_sim_time: True, so without this their clocks never
                # advance (stuck at t=0) and RViz's TF buffer silently fails
                # to resolve any transform -- the robot never renders.
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ],
            output='screen'
        ),

        # RViz with RobotModel + an Image display pre-added per camera topic.
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
    ])
