from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # Get launch configurations
    namespace = context.launch_configurations['namespace']
    serial_port = context.launch_configurations['serial_port']
    remap_cmd_vel = context.launch_configurations['remap_cmd_vel'].lower() in ['true', '1', 'yes']
    sonar_enabled = context.launch_configurations['sonar_enabled'].lower() in ['true', '1', 'yes']
    publish_motors_state = context.launch_configurations['publish_motors_state'].lower() in ['true', '1', 'yes']

    # Create namespace-based prefix
    prefix = f'{namespace}/' if namespace != '' else ''

    # Remapping logic
    from_topic = 'cmd_vel'
    to_topic = 'cmd_vel_not_stamped' if remap_cmd_vel else 'cmd_vel'

    # Create and return the Node
    return [
        Node(
            namespace=namespace,
            package='rosaria2',
            executable='rosaria2_node',
            name='pioneer_p3dx_driver',
            parameters=[{
                'tf_prefix': namespace,
                'odom_frame_id': 'odom',
                'base_frame_id': prefix + 'base_link',
                'bumper_frame_id': 'bumper',
                'sonar_frame_id': prefix + 'sonar',
                'serial_port': serial_port,
                'sonar_enabled': sonar_enabled,
                'publish_sonar': sonar_enabled,
                'publish_sonar_pointcloud2': sonar_enabled,
                'publish_motors_state': publish_motors_state,
            }],
            remappings=[(from_topic, to_topic)],
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('sonar_enabled', default_value='false'),
        DeclareLaunchArgument('publish_motors_state', default_value='true'),
        DeclareLaunchArgument('remap_cmd_vel', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
