from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
def launch_setup(context, *args, **kwargs):
    # Get launch configurations
    pkg = 'bender_base'
    parameters = [os.path.join(
        get_package_share_directory(pkg),
        'params',
        'pioneer.yaml'
    )]

    # Create and return the Node
    return [
        Node(
            namespace='',
            package='rosaria2',
            executable='rosaria2_node',
            name='pioneer_p3dx_driver',
            parameters=[parameters],
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('sonar_enabled', default_value='false'),
        DeclareLaunchArgument('publish_motors_state', default_value='true'),
        DeclareLaunchArgument('remap_cmd_vel', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
