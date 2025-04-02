jdamr@jdamr-pc:~/agv_ws$ /opt/ros/humble/lib/cartographer_ros/cartographer_node   -configuration_directory /home/jdamr/agv_ws/src/Project-AGV-main/stage2_slam/cartographer_agv/config   -configuration_basename agv.lua
[INFO] [1743562138.262733126] [cartographer logger]: I0402 11:48:58.000000  5668 configuration_file_resolver.cc:41] Found '/home/jdamr/agv_ws/src/Project-AGV-main/stage2_slam/cartographer_agv/config/agv.lua' for 'agv.lua'.
[INFO] [1743562138.263739631] [cartographer logger]: I0402 11:48:58.000000  5668 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/map_builder.lua' for 'map_builder.lua'.
[INFO] [1743562138.263900262] [cartographer logger]: I0402 11:48:58.000000  5668 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/map_builder.lua' for 'map_builder.lua'.
[INFO] [1743562138.264146708] [cartographer logger]: I0402 11:48:58.000000  5668 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/pose_graph.lua' for 'pose_graph.lua'.
[INFO] [1743562138.264271856] [cartographer logger]: I0402 11:48:58.000000  5668 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/pose_graph.lua' for 'pose_graph.lua'.
[INFO] [1743562138.264691525] [cartographer logger]: I0402 11:48:58.000000  5668 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/trajectory_builder.lua' for 'trajectory_builder.lua'.
[INFO] [1743562138.264871619] [cartographer logger]: I0402 11:48:58.000000  5668 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/trajectory_builder.lua' for 'trajectory_builder.lua'.
[INFO] [1743562138.265091601] [cartographer logger]: I0402 11:48:58.000000  5668 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/trajectory_builder_2d.lua' for 'trajectory_builder_2d.lua'.
[INFO] [1743562138.265222898] [cartographer logger]: I0402 11:48:58.000000  5668 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/trajectory_builder_2d.lua' for 'trajectory_builder_2d.lua'.
[INFO] [1743562138.265629956] [cartographer logger]: I0402 11:48:58.000000  5668 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/trajectory_builder_3d.lua' for 'trajectory_builder_3d.lua'.
[INFO] [1743562138.265761216] [cartographer logger]: I0402 11:48:58.000000  5668 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/trajectory_builder_3d.lua' for 'trajectory_builder_3d.lua'.
F0402 11:48:58.267215  5668 lua_parameter_dictionary.cc:410] Check failed: 1 == reference_counts_.count(key) (1 vs. 0) Key 'use_odometry' was used the wrong number of times.
[FATAL] [1743562138.268733157] [cartographer logger]: F0402 11:48:58.000000  5668 lua_parameter_dictionary.cc:410] Check failed: 1 == reference_counts_.count(key) (1 vs. 0) Key 'use_odometry' was used the wrong number of times.
*** Check failure stack trace: ***
    @     0xffffa01ed41c  google::LogMessage::Fail()
    @     0xffffa01f46d0  google::LogMessage::SendToLog()
    @     0xffffa01ed0f4  google::LogMessage::Flush()
    @     0xffffa01eeebc  google::LogMessageFatal::~LogMessageFatal()
    @     0xaaaae8b920f0  (unknown)
    @     0xaaaae8b922cc  (unknown)
    @     0xaaaae8bb0ed8  (unknown)
    @     0xaaaae8b7aeec  (unknown)
    @     0xaaaae8ad743c  (unknown)
    @     0xffff9f7d73fc  (unknown)
    @     0xffff9f7d74cc  __libc_start_main
    @     0xaaaae8adabf0 

