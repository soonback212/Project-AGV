/**
 * @file main.cpp
 * @author LDRobot (contact@ldrobot.com)
 * @brief  main process App
 *         This code is only applicable to LDROBOT LiDAR LD00 LD03 LD08 LD14
 * products sold by Shenzhen LDROBOT Co., LTD
 * @version 0.1
 * @date 2021-11-10
 *
 * @copyright Copyright (c) 2021  SHENZHEN LDROBOT CO., LTD. All rights
 * reserved.
 * Licensed under the MIT License (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License in the file LICENSE
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ldlidar_driver.h"
#include <chrono>
#include <memory>
#include <cmath>

using namespace std::chrono_literals;

class LDLidarNode : public rclcpp::Node {
public:
  LDLidarNode() : Node("ldlidar_node") {
    // 파라미터 선언
    this->declare_parameter<std::string>("port_name", "/dev/ttyACM1");
    this->declare_parameter<int>("baudrate", 230400);
    this->declare_parameter<std::string>("frame_id", "laser");

    this->get_parameter("port_name", port_name_);
    this->get_parameter("baudrate", baudrate_);
    this->get_parameter("frame_id", frame_id_);

    // 퍼블리셔
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

    // 드라이버 생성 및 초기화
    driver_ = std::make_unique<ldlidar::LDLidarDriver>();
    driver_->EnableFilterAlgorithnmProcess(true);
    driver_->RegisterGetTimestampFunctional(std::bind(&LDLidarNode::getTimestamp, this));

    if (!driver_->Start(ldlidar::LDType::LD_14, port_name_, baudrate_)) {
      RCLCPP_ERROR(this->get_logger(), "LiDAR 시작 실패. 포트 또는 장치 확인 필요.");
      rclcpp::shutdown();
      return;
    }

    timer_ = this->create_wall_timer(50ms, std::bind(&LDLidarNode::publishScan, this));
    RCLCPP_INFO(this->get_logger(), "LDLiDAR SLAM 노드 시작됨");
  }

private:
  void publishScan() {
    ldlidar::LaserScan scan_data;
    if (driver_->GetLaserScanData(scan_data, 1500) == ldlidar::LidarStatus::NORMAL) {
      auto msg = std::make_shared<sensor_msgs::msg::LaserScan>();
      rclcpp::Time now = this->now();
      msg->header.stamp = now;
      msg->header.frame_id = frame_id_;

      if (scan_data.points.empty()) return;

      float angle_min = scan_data.points.front().angle * M_PI / 180.0;
      float angle_max = scan_data.points.back().angle * M_PI / 180.0;
      size_t point_count = scan_data.points.size();

      msg->angle_min = angle_min;
      msg->angle_max = angle_max;
      msg->angle_increment = (angle_max - angle_min) / point_count;
      msg->time_increment = 0.0; // 사용 가능하면 설정
      msg->scan_time = 0.1;
      msg->range_min = 0.05;
      msg->range_max = 12.0;

      for (const auto& pt : scan_data.points) {
        msg->ranges.push_back(static_cast<float>(pt.distance) / 1000.0);
        msg->intensities.push_back(static_cast<float>(pt.intensity));
      }

      scan_pub_->publish(*msg);
    }
  }

  uint64_t getTimestamp() {
    rclcpp::Time now = this->get_clock()->now();
    return static_cast<uint64_t>(now.nanoseconds() / 1000);  // us
  }

  std::unique_ptr<ldlidar::LDLidarDriver> driver_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string port_name_;
  int baudrate_;
  std::string frame_id_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LDLidarNode>());
  rclcpp::shutdown();
  return 0;
}

