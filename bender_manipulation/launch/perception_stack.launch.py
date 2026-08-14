import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'bender_manipulation'

    camera_time_bridge_node = Node(
        package=pkg_name,
        executable='camera_time_bridge_node.py',
        name='camera_time_bridge_node',
        output='screen',
        emulate_tty=True
    )

    grasp_pick_node = Node(
        package=pkg_name,
        executable='grasp_pick_node.py',
        name='grasp_pick_node',
        output='screen',
        emulate_tty=True
    )

    graspgen_pick_bridge_node = Node(
        package=pkg_name,
        executable='graspgen_pick_bridge_node.py',
        name='graspgen_pick_bridge_node',
        output='screen',
        emulate_tty=True
    )


    sam3_segment_node = Node(
        package=pkg_name,
        executable='sam3_segment_node.py',
        name='sam3_segment_node',
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        camera_time_bridge_node,
        grasp_pick_node,
        graspgen_pick_bridge_node,
        sam3_segment_node
    ])
