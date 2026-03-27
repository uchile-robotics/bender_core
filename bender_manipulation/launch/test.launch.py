#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Nodo de control de Dynamixels
    dynamixel_node = Node(
        package='bender_manipulation',
        executable='dynamixel_node.py',
        name='bender_joint_controller',
        output='screen'
    )

    # Nodo de lectura serial del encoder
    serial_encoder_node = Node(
        package='bender_manipulation',
        executable='serial_encoder_node.py',
        name='serial_encoder_node',
        output='screen',
        parameters=[{
            'port': '/dev/ttyACM0',
            'baudrate': 115200
        }]
    )

    return LaunchDescription([
        dynamixel_node,
        serial_encoder_node
    ])
