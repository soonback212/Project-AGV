cmake_minimum_required(VERSION 3.5)
project(ldlidar_sl_ros2)

# 기본 설정
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 14)

add_compile_options(-Wall -Wextra -Wpedantic)

# 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 헤더 포함
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/include/
)

# 소스 수집
file(GLOB  LDLIDAR_DRI ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/src/*.cpp)

# 실행 파일 등록 (이 이름이 중요!!)
add_executable(ldlidar_node  
  ${CMAKE_CURRENT_SOURCE_DIR}/src/demo.cpp 
  ${LDLIDAR_DRI} 
)
ament_target_dependencies(ldlidar_node rclcpp sensor_msgs geometry_msgs)
target_link_libraries(ldlidar_node pthread)

# 설치 대상 등록
install(TARGETS ldlidar_node
 DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz2 scripts
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

