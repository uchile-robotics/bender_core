from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("bender", package_name="bender_moveit_config").to_moveit_configs()

    pkg_name = 'bender_moveit_config'
    pkg_share = get_package_share_directory(pkg_name)
    
    rviz_config_file = os.path.join(pkg_share,'config','moveit.rviz')

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True}
        ]
    )

    return LaunchDescription([rviz_node])
