from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("bender", package_name="bender_moveit_config").to_moveit_configs()

    # Mismo nodo que generate_moveit_rviz_launch, pero con nivel de log configurable:
    # en info, el display de marcadores interactivos llena la consola.
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=[
            "-d", LaunchConfiguration("rviz_config"),
            "--ros-args", "--log-level", LaunchConfiguration("rviz_log_level"),
        ],
        parameters=[
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        # ros2_control corre en el namespace /bender, así que joint_states sale ahí.
        remappings=[("joint_states", "/bender/joint_states")],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config" / "moveit.rviz"),
        ),
        DeclareLaunchArgument("rviz_log_level", default_value="warn"),
        rviz_node,
    ])
