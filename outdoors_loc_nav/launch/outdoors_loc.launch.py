from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from pathlib import Path
from launch.conditions import IfCondition

#
# Top-level launch file to start outdoors localization + SLAM.
#    adds GPS localization to any robot from the "articubot_one" main package.
#
# Can be run as:
#   ros2 launch outdoors_loc_nav outdoors_loc.launch.py
#
# or, included from the robot's launch file.
#     for example, see Dragger's launch file:
#         https://github.com/slgrobotics/articubot_one/blob/dev/robots/dragger/launch/dragger.localizers.launch.py
#
#     outdoors_loc_nav = include_launch(
#         "outdoors_loc_nav",
#         ['launch', 'outdoors_loc.launch.py'],
#         {
#             'use_sim_time': use_sim_time,
#             'namespace': namespace,
#             'localizer': 'slam_toolbox',   # or 'amcl' or 'map_server' (default)
#             'map': map_yaml_file,          # optional map file for amcl or map_server
#             #'map': '/opt/ros/jazzy/share/nav2_bringup/maps/warehouse.yaml',
#             #'do_odom_tf': 'true'          # whether to publish static "map->odom" TF (default: true)
#         }
#     )
#

def generate_launch_description():

    pkg = FindPackageShare('outdoors_loc_nav').find('outdoors_loc_nav')

    # Launch arguments (can be overridden)
    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    localizer = LaunchConfiguration('localizer', default='map_server')
    map_yaml_file = LaunchConfiguration('map', default='')
    do_odom_tf = LaunchConfiguration('do_odom_tf', default='true')

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

        DeclareLaunchArgument(
            'localizer',
            default_value='map_server',
            description='choose localizer: slam, amcl, or map_server'
        ),

        DeclareLaunchArgument(
            'map',
            default_value='',
            description='path to map yaml file (for amcl or map_server)'
        ),

        DeclareLaunchArgument(
            'do_odom_tf',
            default_value='true',
            description='Run a static transform publisher "odom->base_link" if true'
        ),

        LogInfo(msg=[
            '============ starting OUTDOOR_LOC_NAV  namespace="', namespace,
            '"  use_sim_time=', use_sim_time,
            '"  localizer=', localizer,
            '"  map=', map_yaml_file,
        ]),

        # --- Include navsat_and_ekf.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(pkg) / 'launch' / 'navsat_and_ekf.launch.py')
            ),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
            }.items()
        ),

        # --- Static transform publisher: map -> odom
        # we need it when the "indoors" EKF node (IMU+wheel odometry) is not used
        Node(
            package="tf2_ros",
            namespace=namespace,
            executable="static_transform_publisher",
            name="map_to_odom_tf_pub",
            arguments=[
                '--x', '0.0',     # X translation in meters
                '--y', '0.0',     # Y translation in meters
                '--z', '0.0',     # Z translation in meters
                '--roll', '0.0',  # Roll in radians
                '--pitch', '0.0', # Pitch in radians
                '--yaw', '0.0',   # Yaw in radians (e.g., 90 degrees)
                '--frame-id', 'map', # Parent frame ID
                '--child-frame-id', 'odom' # Child frame ID
            ],
            condition=IfCondition(do_odom_tf)
        ),

        # --- Include slam.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(pkg) / 'launch' / 'slam.launch.py')
            ),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'map': map_yaml_file,
            }.items(),
            condition=IfCondition(PythonExpression(["'", localizer, "' == 'slam_toolbox'"]))
        ),

        # --- Include map_server.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(pkg) / 'launch' / 'map_server.launch.py')
            ),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'map': map_yaml_file,
            }.items(),
            condition=IfCondition(PythonExpression(["'", localizer, "' == 'map_server'"]))
        ),

        # --- Include amcl.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(pkg) / 'launch' / 'amcl.launch.py')
            ),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'map': map_yaml_file,
            }.items(),
            condition=IfCondition(PythonExpression(["'", localizer, "' == 'amcl'"]))
        ),
    ])
