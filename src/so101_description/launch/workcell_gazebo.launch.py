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

    rviz_config_file = os.path.join(
        pkg_path,
        'rviz',
        'workcell_cameras.rviz'
    )

    plug_sdf_file = os.path.join(
        pkg_path,
        'models',
        'plug.sdf'
    )

    socket_sdf_file = os.path.join(
        pkg_path,
        'models',
        'socket.sdf'
    )

    # Second assembly pair (peg into plate), on the right arm's side, where
    # the camera can see it and the right arm can reach it.
    peg_sdf_file = os.path.join(pkg_path, 'models', 'peg.sdf')
    base_plate_sdf_file = os.path.join(pkg_path, 'models', 'base_plate.sdf')

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

        # One spawner for all 5 controllers (separate spawners compete with
        # each other), with a longer switch timeout: the default 5 s is
        # sometimes too short under load, and then no joint states reach TF.
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                'so101_left_arm_controller',
                'so101_left_gripper_controller',
                'so101_right_arm_controller',
                'so101_right_gripper_controller',
                '--controller-manager-timeout', '30',
                '--switch-timeout', '30',
            ],
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

        # The plug is a separate free body, so it can be picked up.
        # Position in the world: (x, y, table top height).
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', plug_sdf_file,
                '-name', 'plug',
                '-x', '-0.12', '-y', '0.04', '-z', '0.76',
            ],
            output='screen'
        ),

        # The socket is static (it never moves). Move it by changing -x/-y.
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', socket_sdf_file,
                '-name', 'socket',
                '-x', '0.0', '-y', '0.10', '-z', '0.76',
            ],
            output='screen'
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-file', peg_sdf_file, '-name', 'peg',
                       '-x', '0.12', '-y', '0.025', '-z', '0.76'],
            output='screen'
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-file', base_plate_sdf_file, '-name', 'base_plate',
                       '-x', '0.14', '-y', '0.09', '-z', '0.76'],
            output='screen'
        ),

        # Bridge the camera images from Gazebo to ROS 2.
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=[
                '/left_wrist_camera/image_raw',
                '/right_wrist_camera/image_raw',
                '/overhead_camera/rgb/image_raw',
                '/overhead_camera/depth/image_raw',
                '/overhead_camera/segmentation/labels_map',
                '/overhead_camera/segmentation/colored_map',
            ],
            output='screen'
        ),

        # camera_info, the depth point cloud and /clock go through
        # parameter_bridge (image_bridge only handles images).
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/left_wrist_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/right_wrist_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/overhead_camera/rgb/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/overhead_camera/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                # Point cloud made by the depth camera ("<topic>/points").
                '/overhead_camera/depth/image_raw/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                # Sim time: every node uses it, so /clock must be bridged.
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ],
            output='screen'
        ),

        # True object poses, for evaluation only: /ground_truth/<name>_pose.
        Node(
            package='so101_description',
            executable='ground_truth_pose_bridge',
            parameters=[{
                'world_name': 'default',
                'tracked_models': ['plug'],
                'use_sim_time': True,
            }],
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
