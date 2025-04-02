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
        self.ser = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=0.1)
        self.get_logger().info(f"시리얼 포트 연결됨: {self.ser.port}")

        self.imu_pub = self.create_publisher(Imu, 'imu', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.timer = self.create_timer(0.05, self.read_serial_data)

        # 위치 및 회전
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0  # 최종 회전각
        self.th_encoder = 0.0
        self.th_imu = 0.0
        self.prev_left = 0
        self.prev_right = 0
        self.last_time = self.get_clock().now()

        # 보정용
        self.bias_sample_count = 100
        self.buffer_acc = []
        self.buffer_gyro = []
        self.acc_bias = [0.0, 0.0, 0.0]
        self.gyro_bias = [0.0, 0.0, 0.0]
        self.calibrated = False

    def read_serial_data(self):
        while self.ser.in_waiting >= 26:
            sync = self.ser.read(2)
            if sync != b'\xF5\xF5':
                continue

            data = self.ser.read(24)
            if len(data) != 24:
                self.get_logger().warn("데이터 길이 부족 - 무시")
                return

            try:
                values = struct.unpack('>hhhhhhhhhhhh', data)
            except struct.error as e:
                self.get_logger().error(f"struct 에러: {e}")
                return

            # 보정 샘플 수집
            if not self.calibrated:
                self.buffer_acc.append(values[0:3])
                self.buffer_gyro.append(values[3:6])
                if len(self.buffer_acc) >= self.bias_sample_count:
                    acc_sum = [sum(x) for x in zip(*self.buffer_acc)]
                    gyro_sum = [sum(x) for x in zip(*self.buffer_gyro)]
                    self.acc_bias = [v / self.bias_sample_count for v in acc_sum]
                    self.gyro_bias = [v / self.bias_sample_count for v in gyro_sum]
                    self.calibrated = True
                    self.get_logger().info(f"IMU 보정 완료: acc_bias={self.acc_bias}, gyro_bias={self.gyro_bias}")
                return

            # 보정 적용
            ax = (values[0] - self.acc_bias[0]) / 1000.0
            ay = (values[1] - self.acc_bias[1]) / 1000.0
            az = (values[2] - self.acc_bias[2]) / 1000.0
            gx = (values[3] - self.gyro_bias[0]) / 1000.0
            gy = (values[4] - self.gyro_bias[1]) / 1000.0
            gz = (values[5] - self.gyro_bias[2]) / 1000.0
            enc_l, enc_r = values[9], values[10]

            # 시간 계산
            curr_time = self.get_clock().now()
            dt = (curr_time - self.last_time).nanoseconds / 1e9
            self.last_time = curr_time

            # 엔코더 거리
            d_left = (enc_l - self.prev_left) / 1000.0
            d_right = (enc_r - self.prev_right) / 1000.0
            self.prev_left = enc_l
            self.prev_right = enc_r

            d = (d_left + d_right) / 2.0

            # === 정지 상태 판단 ===
            is_moving = abs(d_left) > 0.0001 or abs(d_right) > 0.0001

            if is_moving:
                delta_th_encoder = (d_right - d_left) / 0.2
                delta_th_imu = gz * dt

                self.th_encoder += delta_th_encoder
                self.th_imu += delta_th_imu

                alpha = 0.95
                self.th = alpha * self.th_encoder + (1 - alpha) * self.th_imu

                # 위치 업데이트
                self.x += d * math.cos(self.th)
                self.y += d * math.sin(self.th)
            else:
                delta_th_encoder = 0.0
                delta_th_imu = 0.0
                # th 유지 (회전 누적 없음)

            # === IMU 메시지 ===
            imu_msg = Imu()
            imu_msg.header.stamp = curr_time.to_msg()
            imu_msg.header.frame_id = 'imu_link'
            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az
            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz
            imu_msg.orientation = Quaternion()  # orientation 생략

            self.imu_pub.publish(imu_msg)

            # === 오도메트리 메시지 ===
            odom_msg = Odometry()
            odom_msg.header.stamp = curr_time.to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'
            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y
            odom_msg.pose.pose.position.z = 0.0

            q = R.from_euler('z', self.th).as_quat()
            odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            odom_msg.twist.twist.linear.x = d / dt if is_moving else 0.0
            odom_msg.twist.twist.angular.z = delta_th_encoder / dt if is_moving else 0.0

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

