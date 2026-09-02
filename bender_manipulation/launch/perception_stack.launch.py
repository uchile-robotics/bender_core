import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_name = 'bender_manipulation'

    realsense_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ),
        launch_arguments={
            'camera_name': 'camera',
            'camera_namespace': '',
            'align_depth.enable': 'true',
            'pointcloud.enable': 'true',
            'enable_color': 'true',
            'enable_depth': 'true',
            'initial_reset': 'true',
        }.items()
    )

    camera_time_bridge_node = Node(
        package=pkg_name,
        executable='camera_time_bridge_node.py',
        name='camera_time_bridge_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'input_topic': '/camera/depth/color/points',
            'output_topic': '/camera/depth/color/points_stamped'
        }]
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
        emulate_tty=True,
        parameters=[{
            'prompt': 'yellow box'
        }]
    )

    return LaunchDescription([
        realsense_camera_launch,
        camera_time_bridge_node,
        grasp_pick_node,
        graspgen_pick_bridge_node,
        sam3_segment_node
    ])
