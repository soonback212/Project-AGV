from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ldlidar_stl_ros2',
            executable='ldlidar_node',
            name='ldlidar_node',
            output='screen',
            parameters=[{
                'port_name': '/dev/ttyUSB0',
                'port_baudrate': 230400,
                'frame_id': 'laser'
            }]
        )
    ])