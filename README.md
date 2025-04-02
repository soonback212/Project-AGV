jdamr@jdamr-pc:~/agv_ws$ /opt/ros/humble/lib/cartographer_ros/cartographer_node   -configuration_directory /home/jdamr/agv_ws/src/Project-AGV-main/stage2_slam/cartographer_agv/config   -configuration_basename agv.lua
[INFO] [1743561965.736780466] [cartographer logger]: I0402 11:46:05.000000  5550 configuration_file_resolver.cc:41] Found '/home/jdamr/agv_ws/src/Project-AGV-main/stage2_slam/cartographer_agv/config/agv.lua' for 'agv.lua'.
[INFO] [1743561965.737980232] [cartographer logger]: I0402 11:46:05.000000  5550 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/map_builder.lua' for 'map_builder.lua'.
[INFO] [1743561965.738259122] [cartographer logger]: I0402 11:46:05.000000  5550 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/map_builder.lua' for 'map_builder.lua'.
[INFO] [1743561965.738592272] [cartographer logger]: I0402 11:46:05.000000  5550 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/pose_graph.lua' for 'pose_graph.lua'.
[INFO] [1743561965.738793977] [cartographer logger]: I0402 11:46:05.000000  5550 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/pose_graph.lua' for 'pose_graph.lua'.
[INFO] [1743561965.739825075] [cartographer logger]: I0402 11:46:05.000000  5550 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/trajectory_builder.lua' for 'trajectory_builder.lua'.
[INFO] [1743561965.740083706] [cartographer logger]: I0402 11:46:05.000000  5550 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/trajectory_builder.lua' for 'trajectory_builder.lua'.
[INFO] [1743561965.740459060] [cartographer logger]: I0402 11:46:05.000000  5550 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/trajectory_builder_2d.lua' for 'trajectory_builder_2d.lua'.
[INFO] [1743561965.740669339] [cartographer logger]: I0402 11:46:05.000000  5550 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/trajectory_builder_2d.lua' for 'trajectory_builder_2d.lua'.
[INFO] [1743561965.741344657] [cartographer logger]: I0402 11:46:05.000000  5550 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/trajectory_builder_3d.lua' for 'trajectory_builder_3d.lua'.
[INFO] [1743561965.741573603] [cartographer logger]: I0402 11:46:05.000000  5550 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/trajectory_builder_3d.lua' for 'trajectory_builder_3d.lua'.
F0402 11:46:05.751639  5550 lua_parameter_dictionary.cc:410] Check failed: 1 == reference_counts_.count(key) (1 vs. 0) Key 'use_odometry' was used the wrong number of times.
[FATAL] [1743561965.754507170] [cartographer logger]: F0402 11:46:05.000000  5550 lua_parameter_dictionary.cc:410] Check failed: 1 == reference_counts_.count(key) (1 vs. 0) Key 'use_odometry' was used the wrong number of times.
*** Check failure stack trace: ***
    @     0xffff9c68d41c  google::LogMessage::Fail()
    @     0xffff9c6946d0  google::LogMessage::SendToLog()
    @     0xffff9c68d0f4  google::LogMessage::Flush()
    @     0xffff9c68eebc  google::LogMessageFatal::~LogMessageFatal()
    @     0xaaaad1a220f0  (unknown)
    @     0xaaaad1a222cc  (unknown)
    @     0xaaaad1a40ed8  (unknown)
    @     0xaaaad1a0aeec  (unknown)
    @     0xaaaad196743c  (unknown)
    @     0xffff9bc773fc  (unknown)
    @     0xffff9bc774cc  __libc_start_main
    @     0xaaaad196abf0  (unknown)
Aborted (core dumped)

