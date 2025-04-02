jdamr@jdamr-pc:~/agv_ws$ /opt/ros/humble/lib/cartographer_ros/cartographer_node   -configuration_directory /home/jdamr/agv_ws/src/Project-AGV-main/stage2_slam/cartographer_agv/config   -configuration_basename agv.lua
[INFO] [1743562792.612791904] [cartographer logger]: I0402 11:59:52.000000  5927 configuration_file_resolver.cc:41] Found '/home/jdamr/agv_ws/src/Project-AGV-main/stage2_slam/cartographer_agv/config/agv.lua' for 'agv.lua'.
[INFO] [1743562792.613779762] [cartographer logger]: I0402 11:59:52.000000  5927 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/map_builder.lua' for 'map_builder.lua'.
[INFO] [1743562792.613942850] [cartographer logger]: I0402 11:59:52.000000  5927 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/map_builder.lua' for 'map_builder.lua'.
[INFO] [1743562792.614172454] [cartographer logger]: I0402 11:59:52.000000  5927 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/pose_graph.lua' for 'pose_graph.lua'.
[INFO] [1743562792.614301098] [cartographer logger]: I0402 11:59:52.000000  5927 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/pose_graph.lua' for 'pose_graph.lua'.
[INFO] [1743562792.614737399] [cartographer logger]: I0402 11:59:52.000000  5927 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/trajectory_builder.lua' for 'trajectory_builder.lua'.
[INFO] [1743562792.614905968] [cartographer logger]: I0402 11:59:52.000000  5927 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/trajectory_builder.lua' for 'trajectory_builder.lua'.
[INFO] [1743562792.615122053] [cartographer logger]: I0402 11:59:52.000000  5927 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/trajectory_builder_2d.lua' for 'trajectory_builder_2d.lua'.
[INFO] [1743562792.615250142] [cartographer logger]: I0402 11:59:52.000000  5927 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/trajectory_builder_2d.lua' for 'trajectory_builder_2d.lua'.
[INFO] [1743562792.615677647] [cartographer logger]: I0402 11:59:52.000000  5927 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/trajectory_builder_3d.lua' for 'trajectory_builder_3d.lua'.
[INFO] [1743562792.615810754] [cartographer logger]: I0402 11:59:52.000000  5927 configuration_file_resolver.cc:41] Found '/opt/ros/humble/share/cartographer/configuration_files/trajectory_builder_3d.lua' for 'trajectory_builder_3d.lua'.
F0402 11:59:52.617036  5927 lua_parameter_dictionary.cc:410] Check failed: 1 == reference_counts_.count(key) (1 vs. 0) Key 'use_trajectory_builder_2d' was used the wrong number of times.
[FATAL] [1743562792.618466986] [cartographer logger]: F0402 11:59:52.000000  5927 lua_parameter_dictionary.cc:410] Check failed: 1 == reference_counts_.count(key) (1 vs. 0) Key 'use_trajectory_builder_2d' was used the wrong number of times.
*** Check failure stack trace: ***
    @     0xffff9f3ed41c  google::LogMessage::Fail()
    @     0xffff9f3f46d0  google::LogMessage::SendToLog()
    @     0xffff9f3ed0f4  google::LogMessage::Flush()
    @     0xffff9f3eeebc  google::LogMessageFatal::~LogMessageFatal()
    @     0xaaaaaddd20f0  (unknown)
    @     0xaaaaaddd22cc  (unknown)
    @     0xaaaaaddf0ed8  (unknown)
    @     0xaaaaaddbaeec  (unknown)
    @     0xaaaaadd1743c  (unknown)
    @     0xffff9e9d73fc  (unknown)
    @     0xffff9e9d74cc  __libc_start_main
    @     0xaaaaadd1abf0  (unknown)
Aborted (core dumped)

