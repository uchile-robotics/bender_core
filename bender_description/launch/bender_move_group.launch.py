from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("bender", package_name="bender_moveit_config")
        .to_moveit_configs()
    )

    occupancy_map_monitor_yaml = os.path.join(
        moveit_config.package_path, "config", "occupancy_map_monitor.yaml"
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
            # Parámetros de ejecución
            {
                "trajectory_execution": {
                    "allowed_start_tolerance": 0.01,
                    "allowed_execution_duration_scaling": 1.2,
                    "allowed_goal_duration_margin": 0.5,
                    "max_velocity_scaling_factor": 1.0,
                    "max_acceleration_scaling_factor": 1.0,
                }
            },
            # Adapters (para evitar errores de scaling)
            {
                "planning_request_adapter": {
                    "AddTimeOptimalParameterization": {
                        "max_velocity_scaling_factor": 1.0,
                        "max_acceleration_scaling_factor": 1.0,
                    }
                }
            }
        ],
        arguments=[],
    )

    return LaunchDescription([move_group_node])
