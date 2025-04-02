jdamr@jdamr-pc:~/agv_ws$ ros2 run cartographer_ros cartographer_node
-configuration_directory ~/agv_ws/src/Project-AGV-main/stage2_slam/cartographer_agv/config
-configuration_basename agv.lua
F0402 11:38:02.245661  5319 node_main.cpp:103] Check failed: !FLAGS_configuration_directory.empty() -configuration_directory is missing.
*** Check failure stack trace: ***
    @     0xffff8facd41c  google::LogMessage::Fail()
    @     0xffff8fad46d0  google::LogMessage::SendToLog()
    @     0xffff8facd0f4  google::LogMessage::Flush()
    @     0xffff8faceebc  google::LogMessageFatal::~LogMessageFatal()
    @     0xaaaaae397a40  (unknown)
    @     0xffff8f0b73fc  (unknown)
    @     0xffff8f0b74cc  __libc_start_main
    @     0xaaaaae39abf0  (unknown)
[ros2run]: Aborted
-configuration_directory: command not found
-configuration_basename: command not found
