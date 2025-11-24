from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir
from launch_ros.substitutions import FindPackageShare
from pathlib import Path


def generate_launch_description():

    pkg = FindPackageShare('outdoors_loc_nav').find('outdoors_loc_nav')
    params = Path(pkg) / 'params'

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[params / 'slam_toolbox_params.yaml'],
            remappings=[('scan', 'scan')]
        )
    ])
