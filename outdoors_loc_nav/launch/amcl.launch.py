from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetLaunchConfiguration, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EqualsSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

#
# AMCL launch file for outdoors localization.
#
# Runs AMCL alongside the GPS EKF; AMCL TF broadcasting stays disabled.
#
# Also launches the map server and manages both node lifecycles.
#  

def generate_launch_description():

    package_name = 'outdoors_loc_nav'

    # Launch arguments
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')

    # Parameters
    amcl_params_file = PathJoinSubstitution([
        FindPackageShare(package_name),
        'params',
        'amcl_params.yaml'
    ])

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

        # Own these nodes and their lifecycle manager on both Jazzy and Lyrical.
        GroupAction([
            SetLaunchConfiguration(
                'map', PathJoinSubstitution([
                    FindPackageShare(package_name), 'assets', 'maps', 'empty_map.yaml']),
                condition=IfCondition(EqualsSubstitution(map_file, ''))),
            Node(
                package='nav2_map_server', executable='map_server', name='map_server',
                namespace=PathJoinSubstitution(['/', namespace]),
                parameters=[amcl_params_file, {'yaml_filename': map_file,
                                              'use_sim_time': use_sim_time}],
                remappings=remappings, output='screen'),
            Node(
                package='nav2_amcl', executable='amcl', name='amcl',
                namespace=PathJoinSubstitution(['/', namespace]),
                parameters=[amcl_params_file, {'use_sim_time': use_sim_time}],
                remappings=remappings, output='screen'),
        ]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            namespace=PathJoinSubstitution(['/', namespace]),
            parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                         'node_names': ['map_server', 'amcl']}],
            output='screen',
        ),
    ])
