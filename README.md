jdamr@jdamr-pc:~/agv_ws$ /opt/ros/humble/lib/cartographer_ros/cartographer_node   -configuration_directory /home/jdamr/agv_ws/src/Project-AGV-main/stage2_slam/cartographer_agv/config   -configuration_basename agv.lua
[INFO] [1743562575.507038564] [cartographer logger]: I0402 11:56:15.000000  5875 configuration_file_resolver.cc:41] Found '/home/jdamr/agv_ws/src/Project-AGV-main/stage2_slam/cartographer_agv/config/agv.lua' for 'agv.lua'.
[INFO] [1743562575.508004625] [cartographer logger]: I0402 11:56:15.000000  5875 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/map_builder.lua' for 'map_builder.lua'.
[INFO] [1743562575.508178459] [cartographer logger]: I0402 11:56:15.000000  5875 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/map_builder.lua' for 'map_builder.lua'.
[INFO] [1743562575.508396535] [cartographer logger]: I0402 11:56:15.000000  5875 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/pose_graph.lua' for 'pose_graph.lua'.
[INFO] [1743562575.508526128] [cartographer logger]: I0402 11:56:15.000000  5875 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/pose_graph.lua' for 'pose_graph.lua'.
[INFO] [1743562575.509005630] [cartographer logger]: I0402 11:56:15.000000  5875 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/trajectory_builder.lua' for 'trajectory_builder.lua'.
[INFO] [1743562575.509146575] [cartographer logger]: I0402 11:56:15.000000  5875 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/trajectory_builder.lua' for 'trajectory_builder.lua'.
[INFO] [1743562575.509366984] [cartographer logger]: I0402 11:56:15.000000  5875 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/trajectory_builder_2d.lua' for 'trajectory_builder_2d.lua'.
[INFO] [1743562575.509499929] [cartographer logger]: I0402 11:56:15.000000  5875 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/trajectory_builder_2d.lua' for 'trajectory_builder_2d.lua'.
[INFO] [1743562575.509952006] [cartographer logger]: I0402 11:56:15.000000  5875 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/trajectory_builder_3d.lua' for 'trajectory_builder_3d.lua'.
[INFO] [1743562575.510088562] [cartographer logger]: I0402 11:56:15.000000  5875 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/trajectory_builder_3d.lua' for 'trajectory_builder_3d.lua'.
F0402 11:56:15.515255  5875 lua_parameter_dictionary.cc:399] Check failed: HasKey(key) Key 'num_point_clouds' not in dictionary:
{
  fixed_frame_pose_sampling_ratio = 1.000000,
  imu_sampling_ratio = 1.000000,
  imu_topic = "imu",
  landmarks_sampling_ratio = 1.000000,
  laser_scan_topic = "scan",
  lookup_transform_timeout_sec = 0.200000,
  map_builder = {
    collate_by_trajectory = false,
    num_background_threads = 4.000000,
    pose_graph = {
      constraint_builder = {
        ceres_scan_matcher = {
          ceres_solver_options = {
            max_num_iterations = 10.000000,
            num_threads = 1.000000,
            use_nonmonotonic_steps = true,
          },
          occupied_space_weight = 20.000000,
          rotation_weight = 1.000000,
          translation_weight = 10.000000,
        },
        ceres_scan_matcher_3d = {
          ceres_solver_options = {
            max_num_iterations = 10.000000,
            num_threads = 1.000000,
            use_nonmonotonic_steps = false,
          },
          occupied_space_weight_0 = 5.000000,
          occupied_space_weight_1 = 30.000000,
          only_optimize_yaw = false,
          rotation_weight = 1.000000,
          translation_weight = 10.000000,
        },
        fast_correlative_scan_matcher = {
          angular_search_window = 0.523599,
          branch_and_bound_depth = 7.000000,
          linear_search_window = 7.000000,
        },
        fast_correlative_scan_matcher_3d = {
          angular_search_window = 0.261799,
          branch_and_bound_depth = 8.000000,
          full_resolution_depth = 3.000000,
          linear_xy_search_window = 5.000000,
          linear_z_search_window = 1.000000,
          min_low_resolution_score = 0.550000,
          min_rotational_score = 0.770000,
        },
        global_localization_min_score = 0.700000,
        log_matches = true,
        loop_closure_rotation_weight = 100000.000000,
        loop_closure_translation_weight = 11000.000000,
        max_constraint_distance = 15.000000,
        min_score = 0.650000,
        sampling_ratio = 0.300000,
      },
      global_constraint_search_after_n_seconds = 10.000000,
      global_sampling_ratio = 0.003000,
      log_residual_histograms = true,
      matcher_rotation_weight = 1600.000000,
      matcher_translation_weight = 500.000000,
      max_num_final_iterations = 200.000000,
      optimization_problem = {
        acceleration_weight = 110.000000,
        ceres_solver_options = {
          max_num_iterations = 50.000000,
          num_threads = 7.000000,
          use_nonmonotonic_steps = false,
        },
        fix_z_in_3d = false,
        fixed_frame_pose_rotation_weight = 100.000000,
        fixed_frame_pose_tolerant_loss_param_a = 1.000000,
        fixed_frame_pose_tolerant_loss_param_b = 1.000000,
        fixed_frame_pose_translation_weight = 10.000000,
        fixed_frame_pose_use_tolerant_loss = false,
        huber_scale = 10.000000,
        local_slam_pose_rotation_weight = 100000.000000,
        local_slam_pose_translation_weight = 100000.000000,
        log_solver_summary = false,
        odometry_rotation_weight = 100000.000000,
        odometry_translation_weight = 100000.000000,
        rotation_weight = 16000.000000,
        use_online_imu_extrinsics_in_3d = true,
      },
      optimize_every_n_nodes = 90.000000,
    },
    use_trajectory_builder_2d = true,
    use_trajectory_builder_3d = false,
  },
  map_frame = "map",
  num_laser_scans = 1.000000,
  num_multi_echo_laser_scans = 0.000000,
  num_subdivisions_per_laser_scan = 1.000000,
  odom_frame = "odom",
  odometry_sampling_ratio = 1.000000,
  odometry_topic = "odom",
  pose_publish_period_sec = 0.005000,
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  published_frame = "base_link",
  rangefinder_sampling_ratio = 1.000000,
  submap_publish_period_sec = 0.300000,
  tracking_frame = "base_link",
  trajectory_builder = {
    collate_fixed_frame = true,
    collate_landmarks = false,
    trajectory_builder_2d = {
      adaptive_voxel_filter = {
        max_length = 0.500000,
        max_range = 50.000000,
        min_num_points = 200.000000,
      },
      ceres_scan_matcher = {
        ceres_solver_options = {
          max_num_iterations = 20.000000,
          num_threads = 1.000000,
          use_nonmonotonic_steps = false,
        },
        occupied_space_weight = 1.000000,
        rotation_weight = 40.000000,
        translation_weight = 10.000000,
      },
      imu_gravity_time_constant = 10.000000,
      loop_closure_adaptive_voxel_filter = {
        max_length = 0.900000,
        max_range = 50.000000,
        min_num_points = 100.000000,
      },
      max_range = 12.000000,
      max_z = 2.000000,
      min_range = 0.100000,
      min_z = -0.800000,
      missing_data_ray_length = 1.000000,
      motion_filter = {
        max_angle_radians = 0.001745,
        max_distance_meters = 0.200000,
        max_time_seconds = 5.000000,
      },
      num_accumulated_range_data = 1.000000,
      pose_extrapolator = {
        constant_velocity = {
          imu_gravity_time_constant = 10.000000,
          pose_queue_duration = 0.001000,
        },
        imu_based = {
          gravity_constant = 9.806000,
          imu_acceleration_weight = 1.000000,
          imu_rotation_weight = 1.000000,
          odometry_rotation_weight = 1.000000,
          odometry_translation_weight = 1.000000,
          pose_queue_duration = 5.000000,
          pose_rotation_weight = 1.000000,
          pose_translation_weight = 1.000000,
          solver_options = {
            max_num_iterations = 10.000000,
            num_threads = 1.000000,
            use_nonmonotonic_steps = false,
          },
        },
        use_imu_based = false,
      },
      real_time_correlative_scan_matcher = {
        angular_search_window = 0.349066,
        linear_search_window = 0.100000,
        rotation_delta_cost_weight = 0.100000,
        translation_delta_cost_weight = 0.100000,
      },
      submaps = {
        grid_options_2d = {
          grid_type = "PROBABILITY_GRID",
          resolution = 0.050000,
        },
        num_range_data = 90.000000,
        range_data_inserter = {
          probability_grid_range_data_inserter = {
            hit_probability = 0.550000,
            insert_free_space = true,
            miss_probability = 0.490000,
          },
          range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D",
          tsdf_range_data_inserter = {
            maximum_weight = 10.000000,
            normal_estimation_options = {
              num_normal_samples = 4.000000,
              sample_radius = 0.500000,
            },
            project_sdf_distance_to_scan_normal = true,
            truncation_distance = 0.300000,
            update_free_space = false,
            update_weight_angle_scan_normal_to_ray_kernel_bandwidth = 0.500000,
            update_weight_distance_cell_to_hit_kernel_bandwidth = 0.500000,
            update_weight_range_exponent = 0.000000,
          },
        },
      },
      use_imu_data = true,
      use_online_correlative_scan_matching = true,
      voxel_filter_size = 0.025000,
    },
    trajectory_builder_3d = {
      ceres_scan_matcher = {
        ceres_solver_options = {
          max_num_iterations = 12.000000,
          num_threads = 1.000000,
          use_nonmonotonic_steps = false,
        },
        intensity_cost_function_options_0 = {
          huber_scale = 0.300000,
          intensity_threshold = 40.000000,
          weight = 0.500000,
        },
        occupied_space_weight_0 = 1.000000,
        occupied_space_weight_1 = 6.000000,
        only_optimize_yaw = false,
        rotation_weight = 400.000000,
        translation_weight = 5.000000,
      },
      high_resolution_adaptive_voxel_filter = {
        max_length = 2.000000,
        max_range = 15.000000,
        min_num_points = 150.000000,
      },
      imu_gravity_time_constant = 10.000000,
      low_resolution_adaptive_voxel_filter = {
        max_length = 4.000000,
        max_range = 60.000000,
        min_num_points = 200.000000,
      },
      max_range = 60.000000,
      min_range = 1.000000,
      motion_filter = {
        max_angle_radians = 0.004000,
        max_distance_meters = 0.100000,
        max_time_seconds = 0.500000,
      },
      num_accumulated_range_data = 1.000000,
      pose_extrapolator = {
        constant_velocity = {
          imu_gravity_time_constant = 10.000000,
          pose_queue_duration = 0.001000,
        },
        imu_based = {
          gravity_constant = 9.806000,
          imu_acceleration_weight = 1.000000,
          imu_rotation_weight = 1.000000,
          odometry_rotation_weight = 1.000000,
          odometry_translation_weight = 1.000000,
          pose_queue_duration = 5.000000,
          pose_rotation_weight = 1.000000,
          pose_translation_weight = 1.000000,
          solver_options = {
            max_num_iterations = 10.000000,
            num_threads = 1.000000,
            use_nonmonotonic_steps = false,
          },
        },
        use_imu_based = false,
      },
      real_time_correlative_scan_matcher = {
        angular_search_window = 0.017453,
        linear_search_window = 0.150000,
        rotation_delta_cost_weight = 0.100000,
        translation_delta_cost_weight = 0.100000,
      },
      rotational_histogram_size = 120.000000,
      submaps = {
        high_resolution = 0.100000,
        high_resolution_max_range = 20.000000,
        low_resolution = 0.450000,
        num_range_data = 160.000000,
        range_data_inserter = {
          hit_probability = 0.550000,
          intensity_threshold = 40.000000,
          miss_probability = 0.490000,
          num_free_space_voxels = 2.000000,
        },
      },
      use_intensities = false,
      use_online_correlative_scan_matching = false,
      voxel_filter_size = 0.150000,
    },
  },
  trajectory_publish_period_sec = 0.030000,
  use_imu_data = true,
  use_landmarks = false,
  use_nav_sat = false,
  use_odometry = true,
}
[FATAL] [1743562575.520873729] [cartographer logger]: F0402 11:56:15.000000  5875 lua_parameter_dictionary.cc:399] Check failed: HasKey(key) Key 'num_point_clouds' not in dictionary:
{
  fixed_frame_pose_sampling_ratio = 1.000000,
  imu_sampling_ratio = 1.000000,
  imu_topic = "imu",
  landmarks_sampling_ratio = 1.000000,
  laser_scan_topic = "scan",
  lookup_transform_timeout_sec = 0.200000,
  map_builder = {
    collate_by_trajectory = false,
    num_background_threads = 4.000000,
    pose_graph = {
      constraint_builder = {
        ceres_scan_matcher = {
          ceres_solver_options = {
            max_num_iterations = 10.000000,
            num_threads = 1.000000,
            use_nonmonotonic_steps = true,
          },
          occupied_space_weight = 20.000000,
          rotation_weight = 1.000000,
          translation_weight = 10.000000,
        },
        ceres_scan_matcher_3d = {
          ceres_solver_options = {
            max_num_iterations = 10.000000,
            num_threads = 1.000000,
            use_nonmonotonic_steps = false,
          },
          occupied_space_weight_0 = 5.000000,
          occupied_space_weight_1 = 30.000000,
          only_optimize_yaw = false,
          rotation_weight = 1.000000,
          translation_weight = 10.000000,
        },
        fast_correlative_scan_matcher = {
          angular_search_window = 0.523599,
          branch_and_bound_depth = 7.000000,
          linear_search_window = 7.000000,
        },
        fast_correlative_scan_matcher_3d = {
          angular_search_window = 0.261799,
          branch_and_bound_depth = 8.000000,
          full_resolution_depth = 3.000000,
          linear_xy_search_window = 5.000000,
          linear_z_search_window = 1.000000,
          min_low_resolution_score = 0.550000,
          min_rotational_score = 0.770000,
        },
        global_localization_min_score = 0.700000,
        log_matches = true,
        loop_closure_rotation_weight = 100000.000000,
        loop_closure_translation_weight = 11000.000000,
        max_constraint_distance = 15.000000,
        min_score = 0.650000,
        sampling_ratio = 0.300000,
      },
      global_constraint_search_after_n_seconds = 10.000000,
      global_sampling_ratio = 0.003000,
      log_residual_histograms = true,
      matcher_rotation_weight = 1600.000000,
      matcher_translation_weight = 500.000000,
      max_num_final_iterations = 200.000000,
      optimization_problem = {
        acceleration_weight = 110.000000,
        ceres_solver_options = {
          max_num_iterations = 50.000000,
          num_threads = 7.000000,
          use_nonmonotonic_steps = false,
        },
        fix_z_in_3d = false,
        fixed_frame_pose_rotation_weight = 100.000000,
        fixed_frame_pose_tolerant_loss_param_a = 1.000000,
        fixed_frame_pose_tolerant_loss_param_b = 1.000000,
        fixed_frame_pose_translation_weight = 10.000000,
        fixed_frame_pose_use_tolerant_loss = false,
        huber_scale = 10.000000,
        local_slam_pose_rotation_weight = 100000.000000,
        local_slam_pose_translation_weight = 100000.000000,
        log_solver_summary = false,
        odometry_rotation_weight = 100000.000000,
        odometry_translation_weight = 100000.000000,
        rotation_weight = 16000.000000,
        use_online_imu_extrinsics_in_3d = true,
      },
      optimize_every_n_nodes = 90.000000,
    },
    use_trajectory_builder_2d = true,
    use_trajectory_builder_3d = false,
  },
  map_frame = "map",
  num_laser_scans = 1.000000,
  num_multi_echo_laser_scans = 0.000000,
  num_subdivisions_per_laser_scan = 1.000000,
  odom_frame = "odom",
  odometry_sampling_ratio = 1.000000,
  odometry_topic = "odom",
  pose_publish_period_sec = 0.005000,
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  published_frame = "base_link",
  rangefinder_sampling_ratio = 1.000000,
  submap_publish_period_sec = 0.300000,
  tracking_frame = "base_link",
  trajectory_builder = {
    collate_fixed_frame = true,
    collate_landmarks = false,
    trajectory_builder_2d = {
      adaptive_voxel_filter = {
        max_length = 0.500000,
        max_range = 50.000000,
        min_num_points = 200.000000,
      },
      ceres_scan_matcher = {
        ceres_solver_options = {
          max_num_iterations = 20.000000,
          num_threads = 1.000000,
          use_nonmonotonic_steps = false,
        },
        occupied_space_weight = 1.000000,
        rotation_weight = 40.000000,
        translation_weight = 10.000000,
      },
      imu_gravity_time_constant = 10.000000,
      loop_closure_adaptive_voxel_filter = {
        max_length = 0.900000,
        max_range = 50.000000,
        min_num_points = 100.000000,
      },
      max_range = 12.000000,
      max_z = 2.000000,
      min_range = 0.100000,
      min_z = -0.800000,
      missing_data_ray_length = 1.000000,
      motion_filter = {
        max_angle_radians = 0.001745,
        max_distance_meters = 0.200000,
        max_time_seconds = 5.000000,
      },
      num_accumulated_range_data = 1.000000,
      pose_extrapolator = {
        constant_velocity = {
          imu_gravity_time_constant = 10.000000,
          pose_queue_duration = 0.001000,
        },
        imu_based = {
          gravity_constant = 9.806000,
          imu_acceleration_weight = 1.000000,
          imu_rotation_weight = 1.000000,
          odometry_rotation_weight = 1.000000,
          odometry_translation_weight = 1.000000,
          pose_queue_duration = 5.000000,
          pose_rotation_weight = 1.000000,
          pose_translation_weight = 1.000000,
          solver_options = {
            max_num_iterations = 10.000000,
            num_threads = 1.000000,
            use_nonmonotonic_steps = false,
          },
        },
        use_imu_based = false,
      },
      real_time_correlative_scan_matcher = {
        angular_search_window = 0.349066,
        linear_search_window = 0.100000,
        rotation_delta_cost_weight = 0.100000,
        translation_delta_cost_weight = 0.100000,
      },
      submaps = {
        grid_options_2d = {
          grid_type = "PROBABILITY_GRID",
          resolution = 0.050000,
        },
        num_range_data = 90.000000,
        range_data_inserter = {
          probability_grid_range_data_inserter = {
            hit_probability = 0.550000,
            insert_free_space = true,
            miss_probability = 0.490000,
          },
          range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D",
          tsdf_range_data_inserter = {
            maximum_weight = 10.000000,
            normal_estimation_options = {
              num_normal_samples = 4.000000,
              sample_radius = 0.500000,
            },
            project_sdf_distance_to_scan_normal = true,
            truncation_distance = 0.300000,
            update_free_space = false,
            update_weight_angle_scan_normal_to_ray_kernel_bandwidth = 0.500000,
            update_weight_distance_cell_to_hit_kernel_bandwidth = 0.500000,
            update_weight_range_exponent = 0.000000,
          },
        },
      },
      use_imu_data = true,
      use_online_correlative_scan_matching = true,
      voxel_filter_size = 0.025000,
    },
    trajectory_builder_3d = {
      ceres_scan_matcher = {
        ceres_solver_options = {
          max_num_iterations = 12.000000,
          num_threads = 1.000000,
          use_nonmonotonic_steps = false,
        },
        intensity_cost_function_options_0 = {
          huber_scale = 0.300000,
          intensity_threshold = 40.000000,
          weight = 0.500000,
        },
        occupied_space_weight_0 = 1.000000,
        occupied_space_weight_1 = 6.000000,
        only_optimize_yaw = false,
        rotation_weight = 400.000000,
        translation_weight = 5.000000,
      },
      high_resolution_adaptive_voxel_filter = {
        max_length = 2.000000,
        max_range = 15.000000,
        min_num_points = 150.000000,
      },
      imu_gravity_time_constant = 10.000000,
      low_resolution_adaptive_voxel_filter = {
        max_length = 4.000000,
        max_range = 60.000000,
        min_num_points = 200.000000,
      },
      max_range = 60.000000,
      min_range = 1.000000,
      motion_filter = {
        max_angle_radians = 0.004000,
        max_distance_meters = 0.100000,
        max_time_seconds = 0.500000,
      },
      num_accumulated_range_data = 1.000000,
      pose_extrapolator = {
        constant_velocity = {
          imu_gravity_time_constant = 10.000000,
          pose_queue_duration = 0.001000,
        },
        imu_based = {
          gravity_constant = 9.806000,
          imu_acceleration_weight = 1.000000,
          imu_rotation_weight = 1.000000,
          odometry_rotation_weight = 1.000000,
          odometry_translation_weight = 1.000000,
          pose_queue_duration = 5.000000,
          pose_rotation_weight = 1.000000,
          pose_translation_weight = 1.000000,
          solver_options = {
            max_num_iterations = 10.000000,
            num_threads = 1.000000,
            use_nonmonotonic_steps = false,
          },
        },
        use_imu_based = false,
      },
      real_time_correlative_scan_matcher = {
        angular_search_window = 0.017453,
        linear_search_window = 0.150000,
        rotation_delta_cost_weight = 0.100000,
        translation_delta_cost_weight = 0.100000,
      },
      rotational_histogram_size = 120.000000,
      submaps = {
        high_resolution = 0.100000,
        high_resolution_max_range = 20.000000,
        low_resolution = 0.450000,
        num_range_data = 160.000000,
        range_data_inserter = {
          hit_probability = 0.550000,
          intensity_threshold = 40.000000,
          miss_probability = 0.490000,
          num_free_space_voxels = 2.000000,
        },
      },
      use_intensities = false,
      use_online_correlative_scan_matching = false,
      voxel_filter_size = 0.150000,
    },
  },
  trajectory_publish_period_sec = 0.030000,
  use_imu_data = true,
  use_landmarks = false,
  use_nav_sat = false,
  use_odometry = true,
}
*** Check failure stack trace: ***
    @     0xffff933ed41c  google::LogMessage::Fail()
    @     0xffff933f46d0  google::LogMessage::SendToLog()
    @     0xffff933ed0f4  google::LogMessage::Flush()
    @     0xffff933eeebc  google::LogMessageFatal::~LogMessageFatal()
    @     0xaaaad7ab392c  (unknown)
    @     0xaaaad7ab398c  (unknown)
    @     0xaaaad7ab3c9c  (unknown)
    @     0xaaaad7ab3cf4  (unknown)
    @     0xaaaad7a9b25c  (unknown)
    @     0xaaaad79f743c  (unknown)
    @     0xffff929d73fc  (unknown)
    @     0xffff929d74cc  __libc_start_main
    @     0xaaaad79fabf0  (unknown)
Aborted (core dumped)
