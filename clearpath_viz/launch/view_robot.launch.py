from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch Configurations
    robot_model = LaunchConfiguration('robot_model')

    # Launch Arguments
    arg_robot_model = DeclareLaunchArgument(
        'robot_model',
        choices=['a200', 'j100'],
        default_value='a200'
    )

    pkg_clearpath_viz = FindPackageShare('clearpath_viz')
    # Paths
    dir_robot_rviz_config = PathJoinSubstitution([
        pkg_clearpath_viz, 'rviz', robot_model])

    arg_rviz_config = DeclareLaunchArgument(
        name='config',
        default_value='robot.rviz',
    )

    config_rviz = PathJoinSubstitution(
        [dir_robot_rviz_config, LaunchConfiguration('config')]
    )

    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', config_rviz]
    )

    ld = LaunchDescription()
    # Args
    ld.add_action(arg_robot_model)
    ld.add_action(arg_rviz_config)
    # Nodes
    ld.add_action(node_rviz)
    return ld
