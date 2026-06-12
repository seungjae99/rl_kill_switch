import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('kill_switch')
    params_file = os.path.join(pkg_share, 'config', 'kill_switch_params.yaml')

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Node namespace',
    )
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='ROS2 log level (debug/info/warn/error/fatal)',
    )

    node = Node(
        package='kill_switch',
        executable='kill_switch_node',
        name='tug1_kill_switch_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        output='screen',
    )

    return LaunchDescription([namespace_arg, log_level_arg, node])
