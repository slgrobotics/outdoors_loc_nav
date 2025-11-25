*Please visit [main project Wiki pages](https://github.com/slgrobotics/articubot_one/wiki) for details*

## Outdoors Localization Bringup

**See** https://github.com/slgrobotics/articubot_one/wiki/Outdoors-Package-Template

This package launches a full outdoor-capable localization stack:
- NavSat transform
- EKF fusion
- SLAM Toolbox OR Map Server

You need to launch *your robot* and *navigation stack* separately.

Your robot may launch a "*local*" EKF node, to fuse *IMU + EKF* and publish */odometry/local* topic, used indoors.
This package doesn't use or depend on it, but your robot might.

Run:
```
  ros2 launch outdoors_loc_nav outdoors_loc.launch.py
```
or, include (see [Dragger's launch file](https://github.com/slgrobotics/articubot_one/blob/dev/robots/dragger/launch/dragger.localizers.launch.py), for example):
```
    outdoors_loc_nav = include_launch(
        "outdoors_loc_nav",
        ['launch', 'outdoors_loc.launch.py'],
        {
            'use_sim_time': use_sim_time,
            'namespace': namespace
            'localizer': 'map_server',   # or 'amcl' or 'slam_toolbox'  Default: 'map_server'
            'map': map_yaml_file,        # optional, for amcl or map_server (default - "empty" 600x600 map 0.25 m/cell)
            #'map': '/opt/ros/jazzy/share/nav2_bringup/maps/warehouse.yaml',
        }
    )
```

This is how [rqt_graph](https://roboticsbackend.com/rqt-graph-visualize-and-debug-your-ros-graph/) shows relationships between the launched *localization nodes* and *topics*:

<img width="1253" height="810" alt="Screenshot from 2025-11-24 20-30-57" src="https://github.com/user-attachments/assets/eb9a9418-fab5-4a1f-8932-df83c06719a3" />

----------------------------

*Please visit [main project Wiki pages](https://github.com/slgrobotics/articubot_one/wiki) for details*

