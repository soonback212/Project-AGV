jdamr@jdamr-pc:~/agv_ws$ ros2 run cartographer_ros cartographer_node
-configuration_directory /home/jdamr/agv_ws/src/Project-AGV-main/stage2_slam/cartographer_agv/config
-configuration_basename agv.lua
F0402 11:40:36.246186  5362 node_main.cpp:103] Check failed: !FLAGS_configuration_directory.empty() -configuration_directory is missing.
*** Check failure stack trace: ***
    @     0xffffb0ccd41c  google::LogMessage::Fail()
    @     0xffffb0cd46d0  google::LogMessage::SendToLog()
    @     0xffffb0ccd0f4  google::LogMessage::Flush()
    @     0xffffb0cceebc  google::LogMessageFatal::~LogMessageFatal()
    @     0xaaaad5007a40  (unknown)
    @     0xffffb02b73fc  (unknown)
    @     0xffffb02b74cc  __libc_start_main
    @     0xaaaad500abf0  (unknown)
[ros2run]: Aborted
-configuration_directory: command not found
-configuration_basename: command not found
