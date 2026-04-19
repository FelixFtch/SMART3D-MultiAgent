-- Cartographer 2D configuration for RobotCar
-- Hardware: M10P 2D LiDAR + HiPNUC N300 Pro IMU
-- Optimized for corridor/degeneracy resistance
--
-- TF tree (cartographer manages map->odom):
--   map -> odom -> base_link -> laser_link
--                             -> imu_link (optional)

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- Frame configuration
  map_frame = "map",
  tracking_frame = "base_link",       -- IMU data frame_id is base_link
  published_frame = "base_link",      -- No external odom source
  odom_frame = "odom",
  provide_odom_frame = true,          -- Let cartographer generate odom
  publish_frame_projected_to_2d = true,

  -- Sensor sources
  use_odometry = false,               -- No wheel odom yet (set true when DDSM115 ready)
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,                -- M10P publishes LaserScan
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  -- Timing
  lookup_transform_timeout_sec = 0.3,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,     -- 200 Hz pose output
  trajectory_publish_period_sec = 30e-3,

  -- Sampling ratios (keep all data)
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- Use 2D SLAM
MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4  -- RPi5 has 4 cores

---------------------------------------------------------------------------
-- TRAJECTORY BUILDER 2D (scan matching + local SLAM)
---------------------------------------------------------------------------
TRAJECTORY_BUILDER_2D.use_imu_data = true

-- M10P range: 0.15m ~ 12m
TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0

-- Accumulate 1 full scan before matching (M10P ~10Hz)
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- ** CORRIDOR ANTI-DEGENERACY: widen search window **
-- In corridors, the along-corridor direction is ambiguous.
-- A wider linear search window helps the real-time correlative
-- scan matcher find the correct alignment even when the geometry
-- provides weak constraints in one direction.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.2
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(30.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- Ceres scan matcher (fine-tuning after correlative match)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.

-- ** IMU trust: moderately high rotation weight helps in corridors **
-- The IMU prevents wild rotational jumps when LiDAR geometry is ambiguous
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.

-- Motion filter: don't insert too many nodes for small movements
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 3.
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.15
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.5)

-- Submap settings: moderate size for indoor environments
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 80
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05  -- 5cm grid

---------------------------------------------------------------------------
-- POSE GRAPH (global SLAM / loop closure)
---------------------------------------------------------------------------
-- Corridor-friendly: slightly relaxed loop closure to avoid false matches
-- in repetitive corridor geometry
POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 50
POSE_GRAPH.constraint_builder.min_score = 0.6        -- higher = fewer false positives
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

-- Search window for loop closure
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)

-- Sampling ratios for constraint building
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.global_sampling_ratio = 0.003

-- Optimization weights
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5

return options
