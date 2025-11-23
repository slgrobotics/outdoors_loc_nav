from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from pathlib import Path


def generate_launch_description():
    pkg = FindPackageShare('outdoors_loc_nav').find('outdoors_loc_nav')
    params = Path(pkg) / 'params'

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(pkg) / 'launch' / 'localization.launch.py'))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(pkg) / 'launch' / 'slam.launch.py'))
        ),
    ])