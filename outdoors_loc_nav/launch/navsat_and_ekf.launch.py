from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from pathlib import Path

#
# Outdoors localization: NavSat Transform + EKF (GPS + Odom + IMU) launch file.
#
# GPS-to-Map Frame Transformation:
#   The primary purpose is to convert global GPS coordinates into the robot's local map frame.
#   The Transform node needs the odometry pose estimate to know the robot's position
#   relative to its starting point in the map frame.
# Estimating Distance and Position:
#   By fusing the odometry with the IMU heading, navsat_transform_node can accurately
#   calculate the robot's pose in the local frame, even before a GPS fix is received.
#   This allows it to track the robot's movement from the start.
# Providing a Global Position Estimate:
#   Once the Transform node has a GPS fix, the node uses the IMU's heading and
#   the odometry's position to transform the raw GPS coordinates into the local map frame.
# Fusing into State Estimation:
#   The output is an odometry message that contains the fused position in the map frame,
#   which is then sent back to the state estimation node (like ekf_filter_node_navsat)
#   to update the robot's global position estimate. 
#

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
                ('odometry/filtered', 'odometry/global'),  # odom input. "local" here will cause slow drift from initial position
                ("odometry/gps", "odometry/gps"),   # output odom aligned to GPS
                ('gps/filtered', 'gps/filtered')    # output filtered GPS
            ]
        ),

        # -------------------------------
        # EKF (GPS + Odom + IMU)
        #
        # Note: the other EKF node (ekf_filter_node_local) must be run by the main robot package
        #       to fuse wheel odometry + IMU for local movement.
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
