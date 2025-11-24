from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from pathlib import Path

def generate_launch_description():

    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    pkg = FindPackageShare('outdoors_loc_nav').find('outdoors_loc_nav')
    params = Path(pkg) / 'params'

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace for multi-robot deployment'
        ),

        LogInfo(msg=[
            '============ outdoors_loc_nav: starting navsat + EKF  namespace="', namespace,
            '"  use_sim_time=', use_sim_time
        ]),

        # -------------------------------
        # NavSat → map/odom alignment
        # -------------------------------
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[params / 'navsat_transform.yaml',
                        {'use_sim_time': use_sim_time}],
            remappings=[
                ('imu', 'imu/data'),                # sensor IMU input
                ('gps/fix', 'gps/fix'),             # raw GPS fix
                ('odometry/filtered', 'odometry/global'),  # odom input to correct yaw. "local" here will cause slow drift from initial position
                ("odometry/gps", "odometry/gps"),   # output odom aligned to GPS
                ('gps/filtered', 'gps/filtered')    # output filtered GPS
            ]
        ),

        # -------------------------------
        # EKF (GPS + Odom + IMU)
        # -------------------------------
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_navsat',
            output='screen',
            parameters=[params / 'ekf_navsat_params.yaml',
                        {'use_sim_time': use_sim_time}],
            remappings=[
                ('odometry/filtered', 'odometry/global')
            ]
        ),
    ])
