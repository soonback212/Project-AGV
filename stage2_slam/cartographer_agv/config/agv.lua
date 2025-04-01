include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  publish_frame_projected_to_2d = false,
  num_laser_scans = 1,
  laser_scan_topic = "scan",
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  use_imu_data = true,
  imu_topic = "imu",
  odometry_topic = "odom",
  pose_publish_period_sec = 5e-3,
  submap_publish_period_sec = 0.3,
  trajectory_publish_period_sec = 30e-3,
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_odometry = true
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.0
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

return options

