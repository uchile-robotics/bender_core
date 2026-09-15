#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_manipulation = "bender_manipulation"
    pkg_moveit_config = "bender_moveit_config"

    # --- ARGUMENTOS DE LANZAMIENTO ---
    use_rviz = LaunchConfiguration("use_rviz")

    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Lanzar únicamente la instancia de RViz configurada para MoveIt.",
    )

    # --- MOVEIT STACK ---
    static_vj_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(pkg_moveit_config),
                "launch",
                "static_virtual_joint_tfs.launch.py",
            ])
        )
    )

    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare(pkg_moveit_config), "launch", "rsp.launch.py"]
            )
        )
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(pkg_moveit_config),
                "launch",
                "move_group.launch.py",
            ])
        )
    )

    # ÚNICO RVIZ (MoveIt RViz permite ver tanto el planning como agregar displays de PointCloud)
    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(pkg_moveit_config),
                "launch",
                "moveit_rviz.launch.py",
            ])
        ),
        condition=IfCondition(use_rviz),
    )

    # --- CAMARA REALSENSE ---
    realsense_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("realsense2_camera"),
                "launch",
                "rs_launch.py",
            ])
        ),
        launch_arguments={
            "camera_name": "camera",
            "camera_namespace": "",
            "align_depth.enable": "true",
            "pointcloud.enable": "true",
            "enable_color": "true",
            "enable_depth": "true",
            "initial_reset": "true",
        }.items(),
    )

    # --- NODOS DE PERCEPCIÓN Y MANIPULACIÓN ---
    camera_time_bridge_node = Node(
        package=pkg_manipulation,
        executable="camera_time_bridge_node.py",
        name="camera_time_bridge_node",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "input_topic": "/camera/depth/color/points",
            "output_topic": "/camera/depth/color/points_stamped",
        }],
    )

    grasp_pick_node = Node(
        package=pkg_manipulation,
        executable="grasp_pick_node.py",
        name="grasp_pick_node",
        output="screen",
        emulate_tty=True,
    )

    graspgen_pick_bridge_node = Node(
        package=pkg_manipulation,
        executable="graspgen_pick_bridge_node.py",
        name="graspgen_pick_bridge_node",
        output="screen",
        emulate_tty=True,
    )

    sam3_segment_node = Node(
        package=pkg_manipulation,
        executable="sam3_segment_node.py",
        name="sam3_segment_node",
        output="screen",
        emulate_tty=True,
        parameters=[{"prompt": "yellow box"}],
    )

    return LaunchDescription([
        # Argumentos
        declare_use_rviz,
        # MoveIt Core
        static_vj_tf_launch,
        rsp_launch,
        move_group_launch,
        # Cámara y Percepción
        realsense_camera_launch,
        camera_time_bridge_node,
        sam3_segment_node,
        # Nodos de Agarre y Control
        graspgen_pick_bridge_node,
        grasp_pick_node,
        # Instancia única de RViz
        moveit_rviz_launch,
    ])
