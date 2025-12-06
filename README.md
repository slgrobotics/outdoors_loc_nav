*Please visit [main project Wiki pages](https://github.com/slgrobotics/articubot_one/wiki) for details*

## Outdoors Localization Bringup

This package launches a full outdoor-capable localization stack:
- NavSat transform node
- EKF fusion node (IMU, GPS, wheels Odometry)
- one of: *Map Server*, *SLAM Toolbox*, *Cartographer* or *AMCL* localizers
- optionally, a *static transform* node (odom → base_link)

You need to launch *your robot* and *navigation stack* separately.

**Note:** Your robot should launch a *local EKF node* to fuse IMU and wheel odometry and publish the */odometry/local* topic.
It would also publish a *odom → base_link* transform, which is important for the rest of the system.
That’s the typical setup for indoor navigation. This package, except when using Cartographer, neither uses nor depends on that EKF, though your robot relies on it indoors.

### How to use

Clone the repository to your robot's workspace, build (see example [below](https://github.com/slgrobotics/outdoors_loc_nav/blob/main/README.md#testing)).

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
            'localizer': 'map_server',   # 'slam_toolbox' or 'amcl' or 'cartographer' Default: 'map_server'
            'map': map_yaml_file,        # optional, for amcl or map_server (default - "empty" 600x600 map 0.25 m/cell)
            #'map': '/opt/ros/jazzy/share/nav2_bringup/maps/warehouse.yaml'
        }
    )
```

**Note:**
- *Map Server* and *SLAM Toolbox* work fine.
- *AMCL* and *Cartographer* may need work. If you know how to fix it, please submit a pull request.
- for *Cartographer* you MUST start the robot in East (ENU "yaw=0") orientation. Cartographer does not query initial pose,
and there is [no easy way](https://github.com/slgrobotics/outdoors_loc_nav/issues/1) to set it.
- To change robot's orientation in Gazebo, look for '*initial_yaw*' launch argument in [robots/dragger/launch/dragger.drive.launch.py](https://github.com/slgrobotics/articubot_one/blob/dev/robots/dragger/launch/dragger.drive.launch.py)
- The "*orientation_initializer.launch.py*" is my attempt to solve the Cartographer's initial orientation problem by calling its *trajectory services*. While it seems to work, it is still Work in Progress. 

### Nodes and Topics

This is how [rqt_graph](https://roboticsbackend.com/rqt-graph-visualize-and-debug-your-ros-graph/) shows relationships between the launched *localization nodes* and *topics*:

<img width="1253" height="810" alt="Screenshot from 2025-11-24 20-30-57" src="https://github.com/user-attachments/assets/eb9a9418-fab5-4a1f-8932-df83c06719a3" />

### Testing

The best way to try this package is to launch Dragger robot in Gazebo simulation.

**Note:** I mostly develop using ROS Jazzy, but also test everything using Kilted.

Here are the steps (for complete machine setup refer to [this guide](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/ROS-Jazzy/README.md)):
```
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone https://github.com/slgrobotics/articubot_one.git -b dev    # note: use the "dev" branch 
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

### On a real robot

Here is a short [video](https://youtu.be/8MjJq1Rya98) - my [Dragger](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Dragger) robot navigating, using this package for localization ("*map_server*" mode).

### Why Use the Map Server?

In my outdoor experiments, I encounter two distinct navigation scenarios:

1. Moving the robot while staying close to obstacles (e.g., buildings, fences, walls).
2. Moving the robot far from any objects detectable by LiDAR.

In both cases, a NavSat-based localizer is running, so global localization itself is not a problem.

In the *first scenario*, mapping tools such as *SLAM Toolbox* can use LiDAR scans to incrementally build a map of the environment. However, Nav2 goals can only be set *within the boundaries of that map*. Since the map is limited by the LiDAR’s sensing range (typically ~5–12 meters in bright outdoor conditions), long-distance navigation is not feasible in this mode.

The *second scenario* is where SLAM-based mapping fails entirely because there are no nearby environmental features. However, Nav2 still requires a map for building the *Global Costmap* and performing *global planning*. To enable this, I provide an *empty static map*, for example a 600 × 600 cell grid with 0.25 m resolution. This creates a virtual planning area extending about *150 meters* in each direction from the robot’s starting position.

With this setup, I can set a goal anywhere within that area, and Nav2 will initially plan a straight path to it. As the robot moves, the LiDAR and sonars detect real obstacles, which dynamically appear in the *Local Costmap*. The local planner then adjusts the trajectory in real time to avoid those obstacles while still following the global plan.

----------------------------

### Useful links

ROS2 and GPS data:
- https://docs.ros.org/en/jade/api/robot_localization/html/integrating_gps.html - general principles, ROS1 but still good
- https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/GPS.md - how to set up GPS and more links

SLAM Toolbox:
- https://github.com/SteveMacenski/slam_toolbox
- https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html - Navigating while Mapping (SLAM)
- https://automaticaddison.com/navigation-and-slam-using-the-ros-2-navigation-stack/

Cartographer:
- https://github.com/cartographer-project/cartographer - original (not maintained) repository
- https://google-cartographer.readthedocs.io - docs
- https://github.com/ros2/cartographer - ROS2 integration fork, code
- https://google-cartographer-ros.readthedocs.io/en/latest/index.html  - ROS2 integration, docs

Handling maps using Map Server:
- https://automaticaddison.com/building-a-map-of-the-environment-using-slam-ros-2-jazzy/ (scroll down to "*Save the Map*")

----------------------------

*Please visit [main project Wiki pages](https://github.com/slgrobotics/articubot_one/wiki) for details*

