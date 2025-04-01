#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
import os 
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
  use_sim_time = LaunchConfiguration('use_sim_time', default='false')

  # 경로 설정
  agv_pkg_dir = get_package_share_directory('cartographer_agv')
  urdf_path = os.path.join(agv_pkg_dir, 'urdf', 'agv.urdf.xacro')
  rviz_config_file = os.path.join(agv_pkg_dir, 'rviz', 'agv_cartographer.rviz')

  # URDF 읽기
  with open(urdf_path, 'r') as file:
    urdf_desc = file.read()

  robot_param = {'robot_description': urdf_desc}

  # LD LiDAR 노드
  ldlidar_node = Node(
      package='ldlidar_sl_ros2',
      executable='ldlidar_sl_ros2_node',
      name='ldlidar_publisher_ld14',
      output='screen',
      parameters=[
        {'product_name': 'LDLiDAR_LD14'},
        {'laser_scan_topic_name': 'scan'},
        {'point_cloud_2d_topic_name': 'pointcloud2d'},
        {'frame_id': 'base_laser'},
        {'port_name': '/dev/ttyACM0'},
        {'serial_baudrate': 115200},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': False},
        {'angle_crop_min': 135.0},
        {'angle_crop_max': 225.0}
      ]
  )

  # TF: base_link to base_laser
  base_link_to_laser_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_base_laser_ld14',
    arguments=['0', '0', '0.082', '0', '0', '0', 'base_link', 'base_laser']
  )

  # URDF → TF 퍼블리셔
  urdf_to_robot_state = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    output='screen',
    parameters=[robot_param, {'use_sim_time': use_sim_time}],
  )

  return LaunchDescription([
    ldlidar_node,
    base_link_to_laser_tf_node,
    urdf_to_robot_state
  ])
