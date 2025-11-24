from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from pathlib import Path

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
#             'namespace': namespace
#         }
#     )
#

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

        # --- Include localization.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(pkg) / 'launch' / 'localization.launch.py')
            ),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
            }.items()
        ),

        # # --- Include slam.launch.py
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         str(Path(pkg) / 'launch' / 'slam.launch.py')
        #     ),
        #     launch_arguments={
        #         'namespace': namespace,
        #         'use_sim_time': use_sim_time,
        #     }.items()
        # ),

        # --- Include map_server.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(pkg) / 'launch' / 'map_server.launch.py')
            ),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
            }.items()
        ),
    ])
