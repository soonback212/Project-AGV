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
  ldlidar_driver/include/core/ldlidar_datatype.h
  ldlidar_driver/include/core/ldlidar_driver.h
  ldlidar_driver/include/dataprocess/lipkg.h
  ldlidar_driver/include/filter/tofbf.h
  ldlidar_driver/include/logger/log_module/h
  ldlidar_driver/include/networkcom/network_socket_interface_linux.h
  ldlidar_driver/include/serialcom/serial_interface_linux.h
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

