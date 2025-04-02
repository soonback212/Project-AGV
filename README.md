jdamr@jdamr-pc:~/agv_ws$ ros2 run cartographer_ros cartographer_node
--ros-args
-p configuration_directory:=/home/jdamr/agv_ws/src/Project-AGV-main/stage2_slam/cartographer_agv/config
-p configuration_basename:=agv.lua
F0402 11:41:41.143023  5403 node_main.cpp:103] Check failed: !FLAGS_configuration_directory.empty() -configuration_directory is missing.
*** Check failure stack trace: ***
    @     0xffff8490d41c  google::LogMessage::Fail()
    @     0xffff849146d0  google::LogMessage::SendToLog()
    @     0xffff8490d0f4  google::LogMessage::Flush()
    @     0xffff8490eebc  google::LogMessageFatal::~LogMessageFatal()
    @     0xaaaacacd7a40  (unknown)
    @     0xffff83ef73fc  (unknown)
    @     0xffff83ef74cc  __libc_start_main
    @     0xaaaacacdabf0  (unknown)
[ros2run]: Aborted
--ros-args: command not found
-p: command not found
-p: command not found
