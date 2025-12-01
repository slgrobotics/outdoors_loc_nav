from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path
from launch_ros.substitutions import FindPackageShare

#
# Launching "orientation service" and "initial pose publisher":
#    colcon build; ros2 launch outdoors_loc_nav orientation_initializer.launch.py
#
# Service call:
#    ros2 service call /robot/get_orientation std_srvs/srv/Trigger {}
#

def generate_launch_description():

    package_name = 'outdoors_loc_nav'

    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    pkg_share = FindPackageShare(package_name).find(package_name)

    initial_pose_params = str(Path(pkg_share) / 'params' / 'initial_pose.yaml')

    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for initialization'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time (Gazebo)'
        ),

        LogInfo(msg=[
            '[outdoors_loc_nav] Starting orientation initializer.  namespace="',
            namespace, '"  use_sim_time=', use_sim_time
        ]),

        # --- Orientation service ---
        Node(
            package='outdoors_loc_nav',
            executable='orientation_service',
            name='orientation_service',
            namespace=namespace,
            output='screen'
        ),

        # --- Initial pose publisher (auto-queries /robot/get_orientation) ---
        Node(
            package='outdoors_loc_nav',
            executable='initial_pose_pub',
            name='initial_pose_pub',
            namespace=namespace,
            parameters=[initial_pose_params],
            output='screen'
        ),
    ])
