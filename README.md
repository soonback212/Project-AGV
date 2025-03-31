jdamr@jdamr-pc:~/agv_ws$ colcon build
[1.565s] WARNING:colcon.colcon_core.package_identification:Failed to parse ROS package manifest in 'src/Project-AGV-main/stage2_slam/slam_control': Error(s) in package 'src/Project-AGV-main/stage2_slam/slam_control/package.xml':
Error(s):
- The generic dependency on 'geometry_msgs' is redundant with: build_depend, build_export_depend, exec_depend
Starting >>> ldlidar_stl_ros2
Starting >>> motor_serial
Starting >>> slam_control
--- stderr: ldlidar_stl_ros2                                               
In file included from /home/jdamr/agv_ws/src/Project-AGV-main/stage2_slam/ldlidar_stl_ros2/ldlidar_driver/src/core/ldlidar_driver.cpp:21:
/home/jdamr/agv_ws/src/Project-AGV-main/stage2_slam/ldlidar_stl_ros2/ldlidar_driver/include/core/ldlidar_driver.h:27:10: fatal error: serial_interface_linux.h: No such file or directory
   27 | #include "serial_interface_linux.h"
      |          ^~~~~~~~~~~~~~~~~~~~~~~~~~
compilation terminated.
gmake[2]: *** [CMakeFiles/ldlidar_node.dir/build.make:90: CMakeFiles/ldlidar_node.dir/ldlidar_driver/src/core/ldlidar_driver.cpp.o] Error 1
gmake[2]: *** Waiting for unfinished jobs....
/home/jdamr/agv_ws/src/Project-AGV-main/stage2_slam/ldlidar_stl_ros2/src/ldlidar_node.cpp:28:11: fatal error: core/ldlidar_driver.h: No such file or directory
   28 |  #include "core/ldlidar_driver.h"
      |           ^~~~~~~~~~~~~~~~~~~~~~~
compilation terminated.
gmake[2]: *** [CMakeFiles/ldlidar_node.dir/build.make:76: CMakeFiles/ldlidar_node.dir/src/ldlidar_node.cpp.o] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:137: CMakeFiles/ldlidar_node.dir/all] Error 2
gmake: *** [Makefile:146: all] Error 2
---
Failed   <<< ldlidar_stl_ros2 [10.9s, exited with code 2]
Aborted  <<< slam_control [12.1s]                                       
Aborted  <<< motor_serial [12.1s]

Summary: 0 packages finished [14.7s]
  1 package failed: ldlidar_stl_ros2
  2 packages aborted: motor_serial slam_control
  1 package had stderr output: ldlidar_stl_ros2
