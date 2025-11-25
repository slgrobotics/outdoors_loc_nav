from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

#
# AMCL launch file for outdoors localization.
#
# Uses Nav2 AMCL localizer with an empty map. Provides TF between map and odom frames.
#
# The localizer also launches map_server node to serve the map and functions as a map_saver.
#  

def generate_launch_description():

    package_name = 'outdoors_loc_nav'

    # Launch arguments
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')

    # Parameters
    amcl_params_file = PathJoinSubstitution([
        FindPackageShare(package_name),
        'params',
        'amcl_params.yaml'
    ])

    # Nav2 AMCL launch file
    amcl_launch_file = PathJoinSubstitution([
        FindPackageShare('nav2_bringup'),
        'launch',
        'localization_launch.py'
    ])

    # Include AMCL
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(amcl_launch_file),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': amcl_params_file,
        }.items()
    )

    return LaunchDescription([

        # ---- Declare user arguments ----
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for localization'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use gazebo / simulation time'
        ),

        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                get_package_share_directory(package_name),
                'assets',
                'maps',
                'empty_map.yaml'
            ]),
            description='Full path to the map YAML file'
        ),

        # ---- Log startup configuration ----
        LogInfo(msg=[
            '============ [outdoors_loc_nav] Starting AMCL localization ',
            ' namespace=', namespace,
            ' use_sim_time=', use_sim_time,
        ]),

        LogInfo(msg=[
            'params=', amcl_params_file,
            '  map=', map_file
        ]),

        # ---- Include Nav2 localization launch from /opt/ros/jazzy/share/nav2_bringup/ ----
        amcl_launch,
    ])
