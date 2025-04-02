import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 절대 경로 기반 수정 (현재 경로에 맞춤)
    cartographer_dir = '/home/jdamr/agv_ws/src/Project-AGV-main/stage2_slam/cartographer_agv'
    urdf_file = os.path.join(cartographer_dir, 'urdf', 'agv.urdf')
    rviz_config_file = os.path.join(cartographer_dir, 'rviz', 'agv_cartographer.rviz')
    config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(cartographer_dir, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename', default='agv.lua')

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    # LiDAR 드라이버 패키지명은 정확히 유지 (수정 X)
    ldlidar_launch_file_dir = os.path.join('/home/jdamr/agv_ws/install/ldlidar_sl_ros2/share/ldlidar_sl_ros2/launch')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('cartographer_config_dir', default_value=config_dir),
        DeclareLaunchArgument('configuration_basename', default_value=configuration_basename),
        DeclareLaunchArgument('resolution', default_value=resolution),
        DeclareLaunchArgument('publish_period_sec', default_value=publish_period_sec),

        # LiDAR 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ldlidar_launch_file_dir, '/ld14.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # TF: map → odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # TF: odom → base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # Robot State Publisher (URDF 직접 읽기)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': open(urdf_file).read()},
                {'use_sim_time': use_sim_time}
            ]
        ),

        # Cartographer SLAM 노드
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', configuration_basename
            ]
        ),

        # Occupancy Grid 퍼블리시
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(cartographer_dir, 'launch', 'occupancy_grid.launch.py')]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'resolution': resolution,
                'publish_period_sec': publish_period_sec
            }.items(),
        ),

        # IMU 노드
        Node(
            package='slam_control',
            executable='imu_parser_node',
            name='imu_parser_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # 모터 제어 노드
        Node(
            package='slam_control',
            executable='motor_serial_node',
            name='motor_serial_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # RViz 실행
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_file],
            output='screen'
        )
    ])

