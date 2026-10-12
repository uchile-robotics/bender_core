from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

CONTROLLER_NAMESPACE = "bender"


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("bender", package_name="bender_moveit_config").to_moveit_configs()
    launch_dir = moveit_config.package_path / "launch"

    def include(name, **kwargs):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / name)), **kwargs)

    # Hardware mock (bender.urdf.xacro pide hardware:=mock) en el mismo namespace que el robot real.
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=CONTROLLER_NAMESPACE,
        parameters=[str(moveit_config.package_path / "config" / "ros2_controllers.yaml")],
        remappings=[("robot_description", "/robot_description")],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        include("static_virtual_joint_tfs.launch.py"),
        include("rsp.launch.py"),
        include("move_group.launch.py"),
        include("moveit_rviz.launch.py", condition=IfCondition(LaunchConfiguration("use_rviz"))),
        ros2_control_node,
        include("spawn_controllers.launch.py"),
    ])
