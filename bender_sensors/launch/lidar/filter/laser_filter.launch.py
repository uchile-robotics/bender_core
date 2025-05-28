from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = 'bender_sensors'
    parameters = os.path.join(
        get_package_share_directory(pkg),
        'params',
        'lidar',
        'filter',
        'scan_filter_chain.yaml'
    )


    return LaunchDescription([
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='laser_filter_node',
            parameters=[
                {'scan_topic': '/scan_raw'},
                parameters
            ],

            remappings=[
                ('scan', '/scan_raw'),  # entrada
                ('scan_filtered', '/scan')  # salida
            ],
        )
    ])
