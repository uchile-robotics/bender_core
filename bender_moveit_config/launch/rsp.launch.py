from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import SetRemap
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("bender", package_name="bender_moveit_config").to_moveit_configs()
    # ros2_control corre en el namespace /bender, así que joint_states sale ahí.
    return LaunchDescription([
        GroupAction([
            SetRemap("joint_states", "/bender/joint_states"),
            *generate_rsp_launch(moveit_config).entities,
        ])
    ])
