import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    cartographer_dir = get_package_share_directory('cartographer_agv')
    urdf_file = os.path.join(cartographer_dir, 'urdf', 'agv.urdf')  # xacro → urdf로 바꿔줘야 함
    rviz_config_file = os.path.join(cartographer_dir, 'rviz', 'agv_cartographer.rviz')
    config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(cartographer_dir, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename', default='agv.lua')

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    ldlidar_launch_file_dir = os.path.join(get_package_share_directory('ldlidar_stl_ros2'), 'launch')

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('cartographer_config_dir', default_value=config_dir),
        DeclareLaunchArgument('configuration_basename', default_value=configuration_basename),
        DeclareLaunchArgument('resolution', default_value=resolution),
        DeclareLaunchArgument('publish_period_sec', default_value=publish_period_sec),

        # LiDAR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ldlidar_launch_file_dir, '/ld14.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # TF: map -> odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),

        # TF: odom -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf_file],
            output='screen'
        ),

        # Cartographer Node
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

        # Occupancy Grid Map Publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([cartographer_dir, '/launch/occupancy_grid.launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'resolution': resolution,
                'publish_period_sec': publish_period_sec
            }.items(),
        ),

        # IMU Node
        Node(
            package='slam_control',
            executable='imu_parser_node',
            name='imu_parser_node',
            output='screen'
        ),

        # Motor Control
        Node(
            package='slam_control',
            executable='motor_serial_node',
            name='motor_serial_node',
            output='screen'
        ),

        # RViz 별도 실행 (충돌 방지)
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_file],
            output='screen'
        )
    ])
