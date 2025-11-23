from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir
from launch_ros.substitutions import FindPackageShare
from pathlib import Path

def generate_launch_description():
    pkg = FindPackageShare('outdoors_loc_nav').find('outdoors_loc_nav')
    params = Path(pkg) / 'params'

    return LaunchDescription([
        # NavSat → odom
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            parameters=[params / 'navsat_params.yaml'],
            remappings=[
                ('imu/data', '/imu/data'),
                ('gps/fix', '/gps/fix'),
                ('gps/filtered', '/gps/filtered')
            ]
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            parameters=[params / 'ekf_params.yaml'],
            remappings=[
                ('odometry/filtered', '/odometry/local')
            ]
        ),
    ])
