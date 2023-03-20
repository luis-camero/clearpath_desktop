from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

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

    pkg_clearpath_platform_description = FindPackageShare('clearpath_platform_description')
    pkg_clearpath_viz = FindPackageShare('clearpath_viz')

    dir_robot_rviz_config = PathJoinSubstitution([
        pkg_clearpath_viz, 'rviz', robot_model])

    arg_rviz_config = DeclareLaunchArgument(
        name='config',
        default_value='model.rviz',
    )

    config_rviz = PathJoinSubstitution(
        [dir_robot_rviz_config, LaunchConfiguration('config')]
    )

    launch_description = IncludeLaunchDescription(
        PathJoinSubstitution([
            pkg_clearpath_platform_description,
            'launch',
            'description.launch.py'])
    )

    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', config_rviz]
    )

    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    ld = LaunchDescription()
    # Args
    ld.add_action(arg_robot_model)
    ld.add_action(arg_rviz_config)
    # Launches
    ld.add_action(launch_description)
    # Nodes
    ld.add_action(node_rviz)
    ld.add_action(node_joint_state_publisher_gui)
    return ld
