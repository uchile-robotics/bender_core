#!/usr/bin/python3

import os
import yaml
import random
import time
import math
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    pkg_bender_desc = 'bender_description'
    pkg_bender_share = get_package_share_directory(pkg_bender_desc)

    # --- Launch args ---
    world = LaunchConfiguration('world')
    world_path = os.path.join(pkg_bender_share, 'worlds', 'workshop.world')
    declare_world = DeclareLaunchArgument('world', default_value=world_path)
    declare_rviz = DeclareLaunchArgument('rviz', default_value='True')

    urdf_path = PathJoinSubstitution([pkg_bender_share, 'urdf', 'bender.xacro'])
    declare_urdf = DeclareLaunchArgument('urdf', default_value=urdf_path)

    declare_spawn_zone = DeclareLaunchArgument(
        'spawn_zone',
        default_value='mesa_1',
        description='Zona donde spawnear al robot: mesa_1, mesa_2, mesa_3, mesa_4'
    )

    # Carga YAML con posiciones
    spawn_zones = load_yaml(pkg_bender_desc, 'config/spawn_zones.yaml')
    robot_description = {'robot_description': Command(["xacro ", urdf_path])}

    # --- robot_state_publisher ---
    robot_state_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {"use_sim_time": True}]
    )

    # --- Gazebo ---
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments=[('gz_args', [' -r 4 ', world])]
    )

    # --- NUEVO: cálculo de posición de spawn ---
    spawn_zone_default = 'mesa_3'
    zones_file = os.path.join(pkg_bender_share, 'config', 'spawn.yaml')
    with open(zones_file, 'r') as f:
        zones = yaml.safe_load(f)
    zone = zones.get(spawn_zone_default, {'x': 0.0, 'y': 0.0, 'z': 0.1, 'yaw': 0.0})

    random.seed(time.time())
    pos_noise_std = 0.1
    yaw_noise_std = 0.1
    x = zone['x'] + random.uniform(-pos_noise_std, pos_noise_std)
    y = zone['y'] + random.uniform(-pos_noise_std, pos_noise_std)
    z = zone['z']
    yaw = zone['yaw'] + random.uniform(-yaw_noise_std, yaw_noise_std)

    # --- Spawn del robot (ahora con posición y orientación desde YAML) ---
    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        output='both',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'bender',
            '-allow_renaming', 'true',
            '-pause', 'false',
            '-x', str(x),
            '-y', str(y),
            '-z', str(z),
            '-Y', str(yaw),
        ]
    )

    # --- Bridges y controladores ---
    bridge_params = os.path.join(pkg_bender_share, 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}']
    )

    start_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image'],
        remappings=[('/camera/image', '/camera/color/image_raw')]
    )

    # --- Octomap server ---
    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',           # frame del robot
            'sensor_model/max_range': 5.0,     # máximo alcance del sensor
            'sensor_model/min_range': 0.05,
            'sensor_model/hit_prob': 0.7,
            'sensor_model/miss_prob': 0.4,
            'sensor_model/min': 0.12,
            'sensor_model/max': 0.97,
            'resolution': 0.05,                # resolución del octomap
            'publish_free_space': True
        }],
        remappings=[
            ('cloud_in', '/camera/depth/color/points')  # el pointcloud de tu cámara
        ]
    )

    spawners = [
        ["joint_state_broadcaster"],
        ["left_arm_controller"],
        ["right_arm_controller"],
        ["head_controller"],
        ["left_gripper_controller"],
        ["right_gripper_controller"],
        ["mecanum_base_controller"],
    ]
    controller_spawners = [
        Node(package='controller_manager', executable='spawner', arguments=s)
        for s in spawners
    ]

    return LaunchDescription([
        declare_world,
        declare_urdf,
        declare_rviz,
        declare_spawn_zone,
        robot_state_node,
        start_gazebo_server_cmd,
        start_gazebo_ros_spawner_cmd,
        ros_gz_bridge,
        start_gazebo_ros_image_bridge_cmd,
        octomap_server_node,
        *controller_spawners,
    ])
