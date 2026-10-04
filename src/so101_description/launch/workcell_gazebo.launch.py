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

        # One spawner call for all 5 controllers instead of 5 separate
        # processes (separate spawners race each other for the same
        # controller_manager lock), plus a generous --switch-timeout: the
        # default 5s switch timeout can be too short for the hardware
        # interface to finish initializing under load, which fails the
        # whole activation (seen in practice: joint_state_broadcaster
        # failing to activate, which disconnects the whole arm from the TF
        # tree since robot_state_publisher then never gets /joint_states).
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

        # The plug is a genuine free rigid body (so it's actually pickable),
        # not part of the robot URDF: spawned separately from its own SDF
        # file at the same spot it used to occupy when welded to the table.
        # World-frame position = (plug_x, plug_y, table_top_z) from
        # dual_arm_workcell_gazebo.urdf.xacro, since table_link has no
        # rotation or xy-offset relative to world.
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', plug_sdf_file,
                '-name', 'plug',
                '-x', '-0.23', '-y', '0.08', '-z', '0.76',
            ],
            output='screen'
        ),

        # The socket is static (models/socket.sdf has <static>true</static>)
        # but no longer welded into the robot URDF either, specifically so
        # it can be repositioned by just editing the -x/-y/-z args below
        # and relaunching -- no xacro edit or colcon rebuild needed.
        # World-frame position = (socket_x, socket_y, table_top_z) from
        # dual_arm_workcell_gazebo.urdf.xacro, same reasoning as the plug
        # above.
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
                # Depth point cloud, generated automatically by the depth
                # camera plugin as "<topic>/points" (topic set via <topic>
                # in the sensor's xacro -- confirmed with `gz topic -l`).
                '/overhead_camera/depth/image_raw/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                # robot_state_publisher and rviz2 both run with
                # use_sim_time: True, so without this their clocks never
                # advance (stuck at t=0) and RViz's TF buffer silently fails
                # to resolve any transform -- the robot never renders.
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ],
            output='screen'
        ),

        # Ground truth for Phase 5 perception evaluation (never to be read
        # as input to a vision pipeline): see ground_truth_pose_bridge.py
        # for why this needs a small custom node rather than a standard
        # ros_gz_bridge type mapping. Publishes /ground_truth/plug_pose.
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
