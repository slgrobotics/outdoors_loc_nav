-- Copyright 2018 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

-- /* Author: Darby Lim */

-- See turtlebot3_ws/src/turtlebot3/turtlebot3/turtlebot3_cartographer/config/turtlebot3_lds_2d.lua
--     https://github.com/ROBOTIS-GIT/turtlebot3/blob/main/turtlebot3_cartographer/config/turtlebot3_lds_2d.lua
--     /opt/ros/jazzy/share/cartographer_ros/configuration_files/mir-100-mapping.lua (MiR100 Mobile Robot)

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

--
-- See:
--     https://google-cartographer-ros.readthedocs.io/en/stable/configuration.html
--     https://google-cartographer-ros.readthedocs.io/en/latest/tuning.html
--     https://google-cartographer-ros.readthedocs.io/en/latest/going_further.html
--     https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html
--
--     https://chatgpt.com/s/t_692a1fbba6f88191a446113937aa3ff4
--

-- ================================
--   FRAME DEFINITIONS (IMPORTANT)
-- ================================

  map_frame = "map",

  -- Cartographer should track motion in IMU frame
  tracking_frame = "imu_link",

  -- PUBLISH POSE IN BASE FRAME
  published_frame = "odom_",

  odom_frame = "odom",

  -- We already publish odom->base_link via EKF, so:
  provide_odom_frame = false,

  -- publish a pure 2D pose for Nav2
  publish_frame_projected_to_2d = true,

-- ================================
--     SENSOR SOURCES
--
-- See https://google-cartographer-ros.readthedocs.io/en/latest/going_further.html#more-input
--
-- ================================

  use_odometry = false, -- when true, needs remapping: ('odom','odometry/global', 'odometry/local' or 'diff_cont/odom')
  use_pose_extrapolator = false,

  -- Enabling this as we use GPS:
  use_nav_sat = true,  -- when true, needs remapping: ('fix','gps/filtered')

  use_landmarks = false,

  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

-- ================================
--   TIMING
-- ================================

  lookup_transform_timeout_sec = 0.3,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-2,           -- 20 Hz (default 5e-3 - 200 Hz)
  trajectory_publish_period_sec = 30e-3,    -- 30 ms = 33 Hz

  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 0.1,                 -- keep it light
  landmarks_sampling_ratio = 1.0,
}

-- =========================================
--   MAP_BUILDER and TRAJECTORY TUNING
--     - minimal changes from defaults:

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10

-- =========================================
--
-- Further TRAJECTORY BUILDER 2D PARAMETER TUNING
--
--  Override some default parameters,
--  (hints from files in /opt/ros/jazzy/share/cartographer/configuration_files)
--      map_builder.lua
--      map_builder_server.lua
--      pose_graph.lua
--      trajectory_builder_2d.lua
--      trajectory_builder_3d.lua
--      trajectory_builder.lua
-- =========================================

TRAJECTORY_BUILDER.collate_landmarks = false

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 45

-- more points
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.2
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 400

-- slightly slower insertion
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.53
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.493

-- slightly shorter rays
TRAJECTORY_BUILDER_2D.max_range = 15.0

-- wheel odometry is fine
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 20

-- IMU is ok
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 20

-- Laser scan parameters:
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 10.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 10.0
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true 
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

-- =========================================
--    POSE GRAPH OPTIMIZATION
-- =========================================

-- less outliers
POSE_GRAPH.constraint_builder.max_constraint_distance = 5.0
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

-- tune down IMU in optimization
POSE_GRAPH.optimization_problem.acceleration_weight = 0.1 * 1e3
POSE_GRAPH.optimization_problem.rotation_weight = 0.1 * 3e5

-- to ignore wheels in optimization set weights to 0.0
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e3
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 3e3

POSE_GRAPH.optimization_problem.log_solver_summary = true

POSE_GRAPH.optimization_problem.huber_scale = 5e2
-- POSE_GRAPH.optimize_every_n_nodes = 40
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
-- POSE_GRAPH.constraint_builder.log_matches = true

return options
