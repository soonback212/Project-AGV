Traceback (most recent call last):
  File "/opt/ros/humble/share/ament_cmake_core/cmake/core/package_xml_2_cmake.py", line 150, in <module>
    main()
  File "/opt/ros/humble/share/ament_cmake_core/cmake/core/package_xml_2_cmake.py", line 53, in main
    raise e
  File "/opt/ros/humble/share/ament_cmake_core/cmake/core/package_xml_2_cmake.py", line 49, in main
    package = parse_package_string(
  File "/usr/local/lib/python3.10/dist-packages/catkin_pkg/package.py", line 786, in parse_package_string
    raise InvalidPackage('Error(s):%s' % (''.join(['\n- %s' % e for e in errors])), filename)
catkin_pkg.package.InvalidPackage: Error(s) in package '/home/jdamr/agv_ws/src/Project-AGV-main/stage2_slam/ldlidar_stl_ros2/package.xml':
Error(s):
- The generic dependency on 'rclcpp' is redundant with: exec_depend
- The generic dependency on 'sensor_msgs' is redundant with: exec_depend
- The generic dependency on 'std_msgs' is redundant with: exec_depend
CMake Error at /opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package_xml.cmake:95 (message):
  execute_process(/usr/bin/python3
  /opt/ros/humble/share/ament_cmake_core/cmake/core/package_xml_2_cmake.py
  /home/jdamr/agv_ws/src/Project-AGV-main/stage2_slam/ldlidar_stl_ros2/package.xml
  /home/jdamr/agv_ws/build/ldlidar_stl_ros2/ament_cmake_core/package.cmake)
  returned error code 1
Call Stack (most recent call first):
  /opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package_xml.cmake:49 (_ament_package_xml)
  /opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake:63 (ament_package_xml)
  CMakeLists.txt:40 (ament_package)
