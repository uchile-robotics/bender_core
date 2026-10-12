from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

CONTROLLER_NAMESPACE = "bender"


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("bender", package_name="bender_moveit_config").to_moveit_configs()
    moveit_names = moveit_config.trajectory_execution.get(
        "moveit_simple_controller_manager", {}).get("controller_names", [])
    # En moveit_controllers.yaml los nombres llevan el namespace como prefijo.
    controller_names = [name.split("/")[-1] for name in moveit_names] + ["joint_state_broadcaster"]

    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", f"/{CONTROLLER_NAMESPACE}/controller_manager"],
            output="screen",
        )
        for name in controller_names
    ])
