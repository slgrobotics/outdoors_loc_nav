from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

#
# SLAM Toolbox launch file for outdoors localization.
#  

def generate_launch_description():

    package_name = 'outdoors_loc_nav'

    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    slam_toolbox_params_file = PathJoinSubstitution([
        FindPackageShare(package_name), 'params', 'slam_toolbox_params.yaml'
    ])

    slam_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution([
            FindPackageShare('slam_toolbox'),
            'launch',
            'online_async_launch.py'
        ])
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for SLAM'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),

        LogInfo(msg=[
            '============ [outdoors_loc_nav]  starting SLAM Toolbox  namespace="', namespace,
            '"  use_sim_time=', use_sim_time,
            '  params=', slam_toolbox_params_file
        ]),

        IncludeLaunchDescription(
            slam_launch,
            launch_arguments={
                'use_sim_time': use_sim_time,
                'namespace': namespace,
                'slam_params_file': slam_toolbox_params_file
            }.items()
        )
    ])
