from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='a300_00095',
            description='Robot namespace (the prefix on every Husky topic)'),
        DeclareLaunchArgument(
            'config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('husky_health_monitor'), 'config', 'a300_00095.yaml']),
            description='Health check config file'),
        DeclareLaunchArgument(
            'web_port', default_value='-1',
            description='Dashboard port (-1 = use the port in the config file)'),
        Node(
            package='husky_health_monitor',
            executable='health_monitor',
            name='health_monitor',
            namespace=namespace,
            output='screen',
            parameters=[{
                'config_file': LaunchConfiguration('config'),
                'robot_namespace': namespace,
                'web_port': ParameterValue(LaunchConfiguration('web_port'), value_type=int),
            }],
        ),
    ])
