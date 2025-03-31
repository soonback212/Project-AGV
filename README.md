jdamr@jdamr-pc:~/agv_ws$ colcon build
Starting >>> ldlidar_sl_ros2
Starting >>> motor_serial
Starting >>> slam_control
Finished <<< slam_control [7.57s]                                   
Finished <<< motor_serial [7.66s]                                   
--- stderr: ldlidar_sl_ros2                               
/home/jdamr/agv_ws/src/Project-AGV-main/stage2_slam/ldlidar_sl_ros2/src/demo.cpp: In member function ‘void LDLidarNode::publishScan()’:
/home/jdamr/agv_ws/src/Project-AGV-main/stage2_slam/ldlidar_sl_ros2/src/demo.cpp:62:14: error: ‘LaserScan’ is not a member of ‘ldlidar’; did you mean ‘sensor_msgs::msg::LaserScan’?
   62 |     ldlidar::LaserScan scan_data;
      |              ^~~~~~~~~
In file included from /opt/ros/humble/include/sensor_msgs/sensor_msgs/msg/laser_scan.hpp:7,
                 from /home/jdamr/agv_ws/src/Project-AGV-main/stage2_slam/ldlidar_sl_ros2/src/demo.cpp:22:
/opt/ros/humble/include/sensor_msgs/sensor_msgs/msg/detail/laser_scan__struct.hpp:247:7: note: ‘sensor_msgs::msg::LaserScan’ declared here
  247 | using LaserScan =
      |       ^~~~~~~~~
/home/jdamr/agv_ws/src/Project-AGV-main/stage2_slam/ldlidar_sl_ros2/src/demo.cpp:63:35: error: ‘scan_data’ was not declared in this scope
   63 |     if (driver_->GetLaserScanData(scan_data, 1500) == ldlidar::LidarStatus::NORMAL) {
      |                                   ^~~~~~~~~
gmake[2]: *** [CMakeFiles/ldlidar_sl_ros2_node.dir/build.make:76: CMakeFiles/ldlidar_sl_ros2_node.dir/src/demo.cpp.o] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:137: CMakeFiles/ldlidar_sl_ros2_node.dir/all] Error 2
gmake: *** [Makefile:146: all] Error 2
---
Failed   <<< ldlidar_sl_ros2 [28.8s, exited with code 2]

Summary: 2 packages finished [30.1s]
  1 package failed: ldlidar_sl_ros2
  1 package had stderr output: ldlidar_sl_ros2
