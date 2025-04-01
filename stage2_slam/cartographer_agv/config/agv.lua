TRAJECTORY_BUILDER.pure_localization = false
TRAJECTORY_BUILDER.use_imu_data = true
TRAJECTORY_BUILDER.trajectory_builder_2d.use_imu_data = true
TRAJECTORY_BUILDER.trajectory_builder_2d.use_odometry = true

return {
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  laser_scan_topic = "scan",
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  use_imu_data = true,
  imu_topic = "imu",
  odometry_topic = "odom",
  ...
}
