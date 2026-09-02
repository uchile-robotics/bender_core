import os
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue  # <-- NEW IMPORT
from launch import LaunchDescription
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction

def generate_launch_description():

    # Package name
    package_name='bender_description'

    # Launch configurations
    rviz = LaunchConfiguration('rviz')
    urdf = LaunchConfiguration('urdf')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Launch Arguments
    declare_rviz = DeclareLaunchArgument(
        name='rviz', default_value='True',
        description='Opens rviz is set to True')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use sim time if true')

    # Path to default world / URDF
    urdf_path = os.path.join(get_package_share_directory(package_name),'urdf','bender.xacro')

    declare_urdf = DeclareLaunchArgument(
        name='urdf', default_value=urdf_path,
        description='Path to the robot description file')

    robot_description = ParameterValue(Command(['xacro ', urdf]), value_type=str)

    # Create a robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}]
    )

    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'rviz', 'rviz.rviz')
    rviz2 = GroupAction(
        condition=IfCondition(rviz),
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',)]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    # Launch them all!
    return LaunchDescription([
        declare_urdf,
        declare_use_sim_time,
        robot_state_publisher,
        declare_rviz,
        rviz2,
        joint_state_publisher,
    ])
