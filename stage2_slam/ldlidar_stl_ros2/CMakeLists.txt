cmake_minimum_required(VERSION 3.5)
project(ldlidar_stl_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 정확한 헤더 경로들 포함
include_directories(
  include
  ldlidar_driver/include
  ldlidar_driver/include/core
  ldlidar_driver/include/dataprocess
  ldlidar_driver/include/filter
  ldlidar_driver/include/logger
  ldlidar_driver/include/networkcom
  ldlidar_driver/include/serialcom
)

add_executable(ldlidar_node 
  src/ldlidar_node.cpp 
  ldlidar_driver/src/core/ldlidar_driver.cpp
  ldlidar_driver/src/serialcom/serial_interface_linux.cpp  
  ldlidar_driver/src/networkcom/network_socket_interface_linux.cpp  
  ldlidar_driver/src/filter/tofbf.cpp  
  ldlidar_driver/src/logger/log_module.cpp
  ldlidar_driver/src/dataprocess/lipkg.cpp
)

ament_target_dependencies(ldlidar_node
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS
  ldlidar_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

