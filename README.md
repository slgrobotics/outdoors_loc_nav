*Please visit [main project Wiki pages](https://github.com/slgrobotics/articubot_one/wiki) for details*

## Outdoors Localization Bringup

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

## Testing

The best way to try this package is to launch Dragger robot in Gazebo simulation.

**Note:** I mostly develop using ROS Jazzy, but also test everything using Kilted.

Here are the steps (for complete machine setup refer to [this guide](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/ROS-Jazzy/README.md)):
```
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone https://github.com/slgrobotics/articubot_one.git -b dev
git clone https://github.com/slgrobotics/ros_battery_monitoring.git
git clone https://github.com/slgrobotics/scan_to_range.git
git clone https://github.com/slgrobotics/outdoors_loc_nav.git
cd ~/robot_ws

sudo rosdep init    # do it once, if you haven't done it before
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -r -y

colcon build
```
Once the *articubot_one* package is built on the Desktop machine, we can run simulation in Gazebo.
```
source /opt/ros/${ROS_DISTRO}/setup.bash
cd ~/robot_ws
colcon build
source ~/robot_ws/install/setup.bash
ros2 launch articubot_one dragger_sim.launch.py
```
You should see Gazebo and RViz GUI coming up.
You may see a white 150m x 150m square ("empty_map" from *map_server*).
If you zoom a bit out in RViz, you will see aerial map
(The Aerial Map plugin in RViz needs a push - flip the topic's *Reliability Policy* to see the map). 

**Note:** see a [simplified template](https://github.com/slgrobotics/articubot_one/wiki/Outdoors-Package-Template) suggested by ChatGPT.com

----------------------------

*Please visit [main project Wiki pages](https://github.com/slgrobotics/articubot_one/wiki) for details*

