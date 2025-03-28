import rclpy #ROS2 Python 클라이언트 라이브러리
from rclpy.node import Node #ROS2 노드의 기본 클래스
import serial #아두이노와의 시리얼 통신을 위한 pyserial 모듈 , 미설치 시 : pip install pyserial
from geometry_msgs.msg import Twist

# 1단계 : 클래스 정의
class MotorSerialNode(Node):
    def __init__(self):
        super().__init__('motor_serial_node') 
        #부모 클래스 Node를 초기화하면서, 노드 이름을 작성
        self.get_logger().info("motor_serial_node 시작됨")
        #ROS2에서 표준 로그를 출력하는 방식

        self.serial_port = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=0.1)

        self.subscription = self.create_subscription(
            Twist, #메시지 타입(속도 정보) -> geometry_msgs/msg/Twist
            '/cmd_vel', # 구독할 토픽 이름 (ROs2 Teleop 등에서 발행됨)
            self.cmd_vel_callback, # 메시지를 받았을 때 실행할 함수
            10 # 큐 사이즈 (버퍼라고 생각하면 됨)
        )
    def cmd_vel_callback(self, msg):
        # 선속도와 각속도 받아오기
        linear = msg.linear.x
        angular = msg.angular.z

        # 기본값 : 정지
        direction = 5 # Stop
        speed = 0

        # 방향 판단
        if linear > 0.1:
            direction = 1 # GO_FORWARD
            speed = int(min(abs(linear) * 255, 255))
        elif linear < -0.1:
            direction = 2 # GO_BACKWARD
            speed = int(min(abs(linear) * 255, 255))
        elif angular > 0.1:
            direction = 3 # TURN_LEFT
            speed = int(min(abs(angular) * 255, 255))
        elif angular < -0.1:
            direction = 4 # TURN_RIGHT
            speed = int(min(abs(angular) * 255, 255))
        else:
            direction = 5 # STOP
            speed = 0
        
        cmd = bytes([0xF5, 0XF5, 0X51, direction, speed])
        self.serial_port.write(cmd)
        
        self.get_logger().info(f"전송된 명령: {cmd.hex()}")



def main(args=None):
        rclpy.init(args=args)  # ROS2 시스템 초기화
        node = MotorSerialNode()  # 우리 노드 인스턴스 생성
        rclpy.spin(node)  # 콜백 루프 시작 (무한 대기)
        node.destroy_node()  # 종료 시 노드 제거
        rclpy.shutdown()  # ROS2 시스템 종료

if __name__ == '__main__':
    main()
