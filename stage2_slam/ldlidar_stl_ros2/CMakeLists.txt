cmake_minimum_required(VERSION 3.5)
project(ldlidar_stl_ros2)

# 컴파일 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 의존성
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# include 디렉토리 추가
include_directories(
  ldlidar_driver/include
  ldlidar_driver/include/core
)

# 실행 파일 빌드
add_executable(ldlidar_node src/ldlidar_node.cpp ldlidar_driver/src/ldlidar_driver.cpp)

ament_target_dependencies(ldlidar_node
  rclcpp
  sensor_msgs
  std_msgs
)

# install 설정
install(TARGETS
  ldlidar_node
  DESTINATION lib/${PROJECT_NAME}
)

# launch 파일도 설치 (선택)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
