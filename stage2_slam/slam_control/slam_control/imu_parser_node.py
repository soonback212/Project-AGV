import rclpy
from rclpy.node import Node
import serial
import struct
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import tf_transformations
import tf2_ros
import time
import math

class IMUParserNode(Node):
    def __init__(self):
        super().__init__('imu_parser_node')
        self.ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=0.1)
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
        if self.ser.in_waiting >= 29:
            data = self.ser.read(29)
            if data[0] == 0xF5 and data[1] == 0xF5:
                values = struct.unpack('>hhhhhhhhhhhh', data[2:26])
                ax, ay, az = [v / 1000.0 for v in values[0:3]]
                gx, gy, gz = [v / 1000.0 for v in values[3:6]]
                mx, my, mz = [v / 1000.0 for v in values[6:9]]
                enc_l, enc_r = values[9], values[10]

                # IMU 메시지
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = 'imu_link'

                imu_msg.linear_acceleration.x = ax
                imu_msg.linear_acceleration.y = ay
                imu_msg.linear_acceleration.z = az

                imu_msg.angular_velocity.x = gx
                imu_msg.angular_velocity.y = gy
                imu_msg.angular_velocity.z = gz

                imu_msg.orientation = Quaternion()  # orientation은 생략 또는 별도 처리
                self.imu_pub.publish(imu_msg)

                # 오도메트리 계산
                curr_time = self.get_clock().now()
                dt = (curr_time - self.last_time).nanoseconds / 1e9
                self.last_time = curr_time

                d_left = (enc_l - self.prev_left) / 1000.0  # mm to m
                d_right = (enc_r - self.prev_right) / 1000.0
                self.prev_left = enc_l
                self.prev_right = enc_r

                d = (d_left + d_right) / 2.0
                delta_th = (d_right - d_left) / 0.2  # 바퀴 간 거리 20cm

                self.th += delta_th
                self.x += d * math.cos(self.th)
                self.y += d * math.sin(self.th)

                # 오도메트리 메시지
                odom_msg = Odometry()
                odom_msg.header.stamp = curr_time.to_msg()
                odom_msg.header.frame_id = 'odom'
                odom_msg.child_frame_id = 'base_link'

                odom_msg.pose.pose.position.x = self.x
                odom_msg.pose.pose.position.y = self.y
                odom_msg.pose.pose.position.z = 0.0

                q = tf_transformations.quaternion_from_euler(0, 0, self.th)
                odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

                odom_msg.twist.twist.linear.x = d / dt
                odom_msg.twist.twist.angular.z = delta_th / dt

                self.odom_pub.publish(odom_msg)

                # TF 브로드캐스트
                t = TransformStamped()
                t.header.stamp = curr_time.to_msg()
                t.header.frame_id = 'odom'
                t.child_frame_id = 'base_link'
                t.transform.translation.x = self.x
                t.transform.translation.y = self.y
                t.transform.translation.z = 0.0
                t.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

                self.tf_broadcaster.sendTransform(t)
            else:
                self.get_logger().warn("패킷 헤더 불일치 - 무시")

def main(args=None):
    rclpy.init(args=args)
    node = IMUParserNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
