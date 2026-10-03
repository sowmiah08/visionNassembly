from launch import LaunchDescription
from launch_ros.actions import SetParameter
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("so101_dual_arm_workcell", package_name="so101_moveit_config").to_moveit_configs()
    base_ld = generate_move_group_launch(moveit_config)
    return LaunchDescription([SetParameter(name="use_sim_time", value=True)] + base_ld.entities)
