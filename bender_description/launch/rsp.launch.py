import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    package_name = FindPackageShare("bender_description")

    urdf_path = PathJoinSubstitution([package_name, "urdf", "bender.xacro"])

    controllers_yaml_path = PathJoinSubstitution([
        package_name, "config", "controllers", "controllers.yaml"
    ])

    urdf = LaunchConfiguration('urdf')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use sim time if true'
    )

    declare_urdf = DeclareLaunchArgument(
        name='urdf', default_value=urdf_path,
        description='Path to the robot description file'
    )

    robot_description_content = ParameterValue(Command(['xacro ', urdf]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content
        }]
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_content},
            controllers_yaml_path
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    mecanum_base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_base_controller", "--controller-manager", "/controller_manager"],
    )

    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller", "--controller-manager", "/controller_manager"],
    )

    # --- NUEVOS SPAWNERS AGREGADOS ---

    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_controller", "--controller-manager", "/controller_manager"],
    )

    head_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["head_controller", "--controller-manager", "/controller_manager"],
    )

    left_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_gripper_controller", "--controller-manager", "/controller_manager"],
    )

    right_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_gripper_controller", "--controller-manager", "/controller_manager"],
    )

    # 3. Lanzar todo junto
    return LaunchDescription([
        declare_urdf,
        declare_use_sim_time,
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        # mecanum_base_controller_spawner,
        left_arm_controller_spawner,
        right_arm_controller_spawner,
        # head_controller_spawner,
        # left_gripper_controller_spawner,
        # right_gripper_controller_spawner
    ])
