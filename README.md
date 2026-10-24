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

-------------------------

### Contents

- [How to use](#how-to-use)
- [ROS 2 Lyrical and namespaces](#ros-2-lyrical-and-namespaces)
- [Nodes and Topics](#nodes-and-topics)
- [Testing](#testing)
- [On a real robot](#on-a-real-robot)
- [Why Use the Map Server?](#why-use-the-map-server)
- [Useful links](#useful-links)

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
- Cartographer uses the modified Lyrical fork's native IMU heading initialization for fresh 2D maps. The robot can start at any heading when `imu/data` supplies a valid ENU compass orientation.

### Cartographer startup and TF ownership

The default Lua configuration enables `initialize_map_from_imu` with a 30-second
timeout and a 1-second maximum IMU age. Keep the robot stationary until Cartographer
logs **Initial map heading**. No trajectory or scan subscriptions are created while
it waits; invalid or missing orientation causes startup to fail after the timeout.
The IMU must publish a normalized ENU quaternion with matching timestamps, and TF
from its message frame to `tracking_frame` must be available at that timestamp.
An arbitrary gyro startup yaw cannot provide compass alignment.

`imu_heading_offset_rad` defaults to zero. Set it consistently with the desired
map axes and the IMU's magnetic declination correction; avoid applying the same
correction twice. Check alignment against the NavSat heading configuration.
This option does not convert NED orientation to ENU.

Continuous IMU input remains disabled (`TRAJECTORY_BUILDER_2D.use_imu_data = false`),
so `tracking_frame` remains `base_footprint`. Native startup uses IMU orientation
independently and accounts for mounting rotation. If continuous IMU input is
enabled, use a tracking frame colocated with the IMU, typically `imu_link`.

TF ownership in Cartographer mode is:
- The robot's local EKF publishes `odom -> base_link` and `odometry/local`.
- Cartographer publishes `map -> odom` (`published_frame = "odom"`,
  `provide_odom_frame = false`).
- Ground projection uses `base_footprint`, keeping the footprint at map z=0
  even when the EKF places base_link at odom z=0. The robot must provide the
  footprint mounting TF, available at scan timestamps.
- The global GPS EKF continues publishing `odometry/global`, with TF publication
  disabled by the top-level launch. Other localizer modes retain its existing
  TF behavior. Standalone NavSat/EKF launch defaults to publishing TF; pass
  `global_ekf_publish_tf:=false` when another node owns `map -> odom`.

The old orientation initializer is no longer launched automatically. Its trajectory
finish/start sequence must not run alongside native fresh-map initialization.
Its explicit service/RViz reset workflow remains available for separate use.
Native heading initialization supports fresh maps only: disable it for saved-state
localization, manual trajectory startup, or another configured initial pose. Saved
map poses must be supplied in that map's coordinates through trajectory services.
The `map` argument selects YAML maps for AMCL/map-server modes; it does not load a
Cartographer `.pbstream`.

Select a robot-specific configuration through the top-level launch:

```bash
ros2 launch outdoors_loc_nav outdoors_loc.launch.py localizer:=cartographer \
  cartographer_config_dir:=/absolute/path/to/robot/config \
  configuration_basename:=cartographer_lds_2d.lua
```

The Lua startup/projection options require the updated Cartographer and
Cartographer ROS overlay to be built and sourced before launching this package.
After rebuilding, restart Cartographer and dependent navigation/visualization nodes.
Validate starts facing North and West, footprint height at zero, one publisher per
TF edge, and startup failure when no valid IMU orientation arrives.

### ROS 2 Lyrical and namespaces

The AMCL launch starts its own map server and lifecycle manager, so it does not
rely on distribution-specific lifecycle behavior in Nav2's localization launch.
The map-server mode also manages its own lifecycle.

The `namespace` argument selects the absolute robot namespace (`dragger` and
`/dragger` both select `/dragger`; the default selects `/`). It can be used from
Dragger's enclosing namespace wrapper without duplicating the namespace.
Parameter files use node-specific wildcard selectors to work at either level.
Sensor topics, Cartographer services, and TF topics resolve within that namespace;
the robot's sensor and local odometry bringup must use the same namespace and TF topics.
Frame IDs and GPS/EKF tuning are unchanged.

The optional legacy Cartographer initializer waits for each service response before
continuing, checks status codes, and uses the selected simulation clock.

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

### Conservative GPS weighting on Dragger

Dragger's live sensor launcher sets the NMEA driver's `epe_quality1` to `8.0`
instead of its default `4.0`. For standard GGA fixes without receiver-provided
GST errors, the driver computes horizontal variance as `(HDOP * EPE)^2`.
This doubles the reported standard deviation and quadruples variance, reducing
GPS influence without changing coordinates or update rate. GST error estimates
override this fallback; verify `/gps/fix` covariance after restarting. Other
robots and Dragger's simulator retain their existing driver settings.

The global EKF uses `odom1_pose_rejection_threshold: 3.0` on `odometry/gps`.
This shared outdoor configuration rejects exceptional GPS innovations using
Mahalanobis distance and the combined measurement/prediction uncertainty; it is
not a three-meter cutoff. With Dragger's recorded horizontal variance increased
from approximately 72-88 to 288-350 m², the cutoff is roughly 51-56 m when
prediction uncertainty is small. It grows as prediction uncertainty grows.
Smooth GPS wandering can still pass, and valid large corrections after an
outage can also be rejected. Compare stationary drift, driving response, and
GPS reacquisition before relying on the new tuning.

After deploying both packages to the robot, rebuild and restart the full live
launch (these settings are read at startup):

```bash
cd /home/ros/robot_ws
colcon build --packages-select articubot_one outdoors_loc_nav
source install/setup.bash
ros2 launch articubot_one dragger.launch.py
```

Check the running values in another sourced terminal:

```bash
ros2 param get /nmea_navsat_driver epe_quality1
ros2 param get /ekf_filter_node_navsat odom1_pose_rejection_threshold
ros2 topic echo /gps/fix --once
```

Expect `8.0` and `3.0`. At similar HDOP to the baseline recording, expect
horizontal GPS covariance around 288-350 m² unless GST supplies the errors.
For a baseline comparison, restore `epe_quality1` to `4.0` and remove the GPS
rejection threshold, then rebuild and restart. Local EKF and IMU tuning are
unchanged.

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

