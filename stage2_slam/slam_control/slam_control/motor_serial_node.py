import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist

# 명령 코드 정의
GO_FORWARD  = 1
GO_BACKWARD = 2
TURN_LEFT   = 3
TURN_RIGHT  = 4
STOP        = 5

class MotorSerialNode(Node):
    def __init__(self):
        super().__init__('motor_serial_node')
        self.get_logger().info("Stage2: motor_serial_node 시작됨")

        # 시리얼 포트 연결
        self.serial_port = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=0.1)

        # /cmd_vel 구독
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        direction = STOP
        speed = 0

        if linear > 0.1:
            direction = GO_FORWARD
            speed = int(min(abs(linear) * 255, 255))
        elif linear < -0.1:
            direction = GO_BACKWARD
            speed = int(min(abs(linear) * 255, 255))
        elif angular > 0.1:
            direction = TURN_LEFT
            speed = int(min(abs(angular) * 255, 255))
        elif angular < -0.1:
            direction = TURN_RIGHT
            speed = int(min(abs(angular) * 255, 255))
        else:
            direction = STOP
            speed = 0

        # 바이너리 명령 전송
        cmd = bytes([0xF5, 0xF5, 0x51, direction, speed])
        self.serial_port.write(cmd)

        self.get_logger().info(f"전송: {cmd.hex()}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
