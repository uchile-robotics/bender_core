import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_name = "bender_description"
    pkg_share = get_package_share_directory(pkg_name)

    # Launch args
    rviz = LaunchConfiguration("rviz")
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="True", description="Launch RViz"
    )

    world = LaunchConfiguration("world")
    world_path = os.path.join(pkg_share, "worlds", "workshop.world")
    declare_world = DeclareLaunchArgument("world", default_value=world_path)

    urdf_path = PathJoinSubstitution([pkg_share, "urdf", "bender.xacro"])
    declare_urdf = DeclareLaunchArgument("urdf", default_value=urdf_path)

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")

    # robot_description
    robot_description_content = ParameterValue(
        Command(["xacro ", urdf_path, " hardware:=sim"]), value_type=str
    )
    robot_description = {"robot_description": robot_description_content}

    # robot_state_publisher
    robot_state_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "odom"],
    )

    # Gazebo
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments=[("gz_args", [" -r  4 ", world])],
    )

    start_gazebo_ros_spawner_cmd = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "bender",
            "-allow_renaming",
            "true",
            "-pause",
            "false",
        ],
    )

    # Gazebo-ROS bridge
    bridge_params = os.path.join(pkg_share, "config", "gz_bridge.yaml")
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["--ros-args", "-p", f"config_file:={bridge_params}"],
    )

    # image bridge
    start_gazebo_ros_image_bridge_cmd = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image"],
        remappings=[("/camera/image", "/camera/color/image_raw")],
    )

    controller_names = [
        "joint_state_broadcaster",
        "pioneer_base_controller",
        "left_shoulder_controller",
        "right_shoulder_controller",
        "left_arm_controller",
        "right_arm_controller",
        "head_controller",
        "left_gripper_controller",
        "right_gripper_controller",
    ]
    spawners = [
        Node(package="controller_manager", executable="spawner", arguments=[name])
        for name in controller_names
    ]

    return LaunchDescription(
        [
            declare_world,
            declare_urdf,
            declare_rviz,
            declare_use_sim_time,
            start_gazebo_server_cmd,
            start_gazebo_ros_spawner_cmd,
            ros_gz_bridge,
            start_gazebo_ros_image_bridge_cmd,
            robot_state_node,
            static_tf,
            *spawners,
        ]
    )
