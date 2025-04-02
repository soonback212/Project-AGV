ros2 run cartographer_ros cartographer_node \
  --ros-args \
  -p configuration_directory:=/home/jdamr/agv_ws/src/Project-AGV-main/stage2_slam/cartographer_agv/config \
  -p configuration_basename:=agv.lua
