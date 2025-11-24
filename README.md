*Please visit [main project Wiki pages](https://github.com/slgrobotics/articubot_one/wiki) for details*

## Outdoors Localization Bringup

**See** https://github.com/slgrobotics/articubot_one/wiki/Outdoors-Package-Template

This package launches a full outdoor-capable localization stack:
- NavSat transform
- EKF fusion
- SLAM Toolbox OR Map Server

You need to launch *your robot* and *navigation stack* separately.

Your robot may launch a "local" EKF node, to fuse IMU + EKF and publish */odometry/local* topic, used indoors. This package doesn't use or depend on it.

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
        }
    )


```
