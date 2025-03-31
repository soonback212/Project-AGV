
jdamr@jdamr-pc:~/agv_ws$ colcon build
[2.721s] WARNING:colcon.colcon_core.package_identification:Failed to parse ROS package manifest in 'src/Project-AGV-main/stage2_slam/slam_control': Error(s) in package 'src/Project-AGV-main/stage2_slam/slam_control/package.xml':
Error(s):
- The generic dependency on 'geometry_msgs' is redundant with: build_depend, build_export_depend, exec_depend
Starting >>> ldlidar_stl_ros2
Starting >>> motor_serial
Starting >>> slam_control
--- stderr: ldlidar_stl_ros2                                         
CMake Error at CMakeLists.txt:21 (add_executable):
  Cannot find source file:

    ldlidar_driver/src/ldlidar_driver.cpp

  Tried extensions .c .C .c++ .cc .cpp .cxx .cu .mpp .m .M .mm .ixx .cppm .h
  .hh .h++ .hm .hpp .hxx .in .txx .f .F .for .f77 .f90 .f95 .f03 .hip .ispc


CMake Error at CMakeLists.txt:21 (add_executable):
  No SOURCES given to target: ldlidar_node


CMake Generate step failed.  Build files cannot be regenerated correctly.
---
Failed   <<< ldlidar_stl_ros2 [4.41s, exited with code 1]
Aborted  <<< motor_serial [5.58s]                                     
Aborted  <<< slam_control [5.55s]

Summary: 0 packages finished [8.83s]
  1 package failed: ldlidar_stl_ros2
  2 packages aborted: motor_serial slam_control
  1 package had stderr output: ldlidar_stl_ros2
