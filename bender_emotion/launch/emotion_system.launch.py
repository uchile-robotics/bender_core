#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo Action Server

        # Nodo que publica emocione
        Node(
            package='bender_emotion',
            executable='emotion_publisher_node',
            name='emotion_publisher_node',
            output='screen'
        ),

        # Nodo que envía emociones por puerto serial
        Node(
            package='bender_emotion',
            executable='emo_serial_propio',
            name='emotion_sender',
            output='screen',
            parameters=[{
                'serial_port': '/dev/head_ino',
                'baudrate': 115200
            }]
        ),

        # Nodo publicar movimiento cuello
        Node(
            package='bender_emotion',
            executable='neck',
            name='neck_publisher',
            output='screen',
            parameters=[{
                'device_name': '/dev/cuello',
                'baudrate': 1000000
            }]
        ),
        Node(
            package='bender_emotion',  
            executable='action_server',
            name='emotion_action_server',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
    ])
