from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    params = PathJoinSubstitution([
        get_package_share_directory("bender_sensors"),
        "params",
        "lidar",
        "filter", 
        "scan_filter_chain.yaml"
    ])
    return LaunchDescription([
        Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            remappings=[
                ("scan", "/scan_raw"),
                ("scan_filtered", "/scan"),
            ],
            parameters=[
                params
            ],
        )
    ])
