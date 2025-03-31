import rclpy
from rclpy.node import Node
import serial
import struct
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from scipy.spatial.transform import Rotation as R
import tf2_ros
import math

class IMUParserNode(Node):
    def __init__(self):
        super().__init__('imu_parser_node')
        self.ser = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=0.1)  # 포트 확인 필요
        self.get_logger().info(f"시리얼 포트 연결됨: {self.ser.port}")

        self.imu_pub = self.create_publisher(Imu, 'imu', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.timer = self.create_timer(0.05, self.read_serial_data)

        self.prev_left = 0
        self.prev_right = 0
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

    def read_serial_data(self):
        while self.ser.in_waiting >= 26:
            sync = self.ser.read(2)
            if sync != b'\xF5\xF5':
                continue  # 헤더 일치할 때까지 반복

            data = self.ser.read(24)
            if len(data) != 24:
                self.get_logger().warn("데이터 길이 부족 - 무시")
                return

            try:
                values = struct.unpack('>hhhhhhhhhhhh', data)
            except struct.error as e:
                self.get_logger().error(f"struct 에러: {e}")
                return

            ax, ay, az = [v / 1000.0 for v in values[0:3]]
            gx, gy, gz = [v / 1000.0 for v in values[3:6]]
            mx, my, mz = [v / 1000.0 for v in values[6:9]]
            enc_l, enc_r = values[9], values[10]

            # 로그로 값 확인
            self.get_logger().info(f"[IMU] acc=({ax:.2f}, {ay:.2f}, {az:.2f})  gyro=({gx:.2f}, {gy:.2f}, {gz:.2f})  enc=({enc_l}, {enc_r})")

            # === IMU 메시지 ===
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'
            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az
            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz
            imu_msg.orientation = Quaternion()  # orientation 생략 (필요 시 보정 가능)

            self.imu_pub.publish(imu_msg)

            # === 오도메트리 계산 ===
            curr_time = self.get_clock().now()
            dt = (curr_time - self.last_time).nanoseconds / 1e9
            self.last_time = curr_time

            d_left = (enc_l - self.prev_left) / 1000.0  # mm → m
            d_right = (enc_r - self.prev_right) / 1000.0
            self.prev_left = enc_l
            self.prev_right = enc_r

            d = (d_left + d_right) / 2.0
            delta_th = (d_right - d_left) / 0.2  # 바퀴 간 거리 20cm 기준
            self.th += delta_th
            self.x += d * math.cos(self.th)
            self.y += d * math.sin(self.th)

            # === 오도메트리 메시지 ===
            odom_msg = Odometry()
            odom_msg.header.stamp = curr_time.to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'
            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y
            odom_msg.pose.pose.position.z = 0.0

            r = R.from_euler('z', self.th)
            q = r.as_quat()  # [x, y, z, w]
            odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            odom_msg.twist.twist.linear.x = d / dt
            odom_msg.twist.twist.angular.z = delta_th / dt

            self.odom_pub.publish(odom_msg)

            # === TF 브로드캐스트 ===
            t = TransformStamped()
            t.header.stamp = curr_time.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

            self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = IMUParserNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

