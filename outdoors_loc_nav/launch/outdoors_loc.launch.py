from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from pathlib import Path


def generate_launch_description():

    pkg = FindPackageShare('outdoors_loc_nav').find('outdoors_loc_nav')

    # Launch arguments (can be overridden)
    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

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
            '============ starting OUTDOOR_LOC_NAV  namespace="', namespace,
            '"  use_sim_time=', use_sim_time
        ]),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(pkg) / 'launch' / 'localization.launch.py'))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(pkg) / 'launch' / 'slam.launch.py'))
        ),
    ])