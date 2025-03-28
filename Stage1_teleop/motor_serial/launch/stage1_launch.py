from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_serial',
            executable='motor_serial_node',
            name='motor_serial_node',
            output='screen'
        ),
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop',
            output='screen',
            prefix='xterm -e',  # 키보드 입력을 받을 수 있게 터미널 띄움
            parameters=[{'use_sim_time': False}]
        )
    ])
