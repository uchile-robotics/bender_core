#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include_moveit_launch(moveit_config_pkg, launch_file, condition=None):
    kwargs = {}
    if condition is not None:
        kwargs["condition"] = condition

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_pkg, "launch", launch_file)
        ),
        **kwargs,
    )


def generate_launch_description():
    moveit_config_pkg = get_package_share_directory("bender_moveit_config")
    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch MoveIt RViz.",
            ),
            include_moveit_launch(
                moveit_config_pkg,
                "static_virtual_joint_tfs.launch.py",
            ),
            include_moveit_launch(
                moveit_config_pkg,
                "rsp.launch.py",
            ),
            include_moveit_launch(
                moveit_config_pkg,
                "move_group.launch.py",
            ),
            include_moveit_launch(
                moveit_config_pkg,
                "moveit_rviz.launch.py",
                condition=IfCondition(use_rviz),
            ),
        ]
    )
