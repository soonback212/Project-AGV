/**
 * @file main.cpp
 * @author LDRobot (support@ldrobot.com)
 * @brief  main process App
 *         This code is only applicable to LDROBOT LiDAR LD06 products 
 * sold by Shenzhen LDROBOT Co., LTD    
 * @version 0.1
 * @date 2021-10-28
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

#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "core/ldlidar_driver.h"
#include "core/ldlidar_datatype.h"

using namespace std::chrono_literals;

class LDLidarNode : public rclcpp::Node {
public:
  LDLidarNode()
  : Node("ldlidar_node") {
    RCLCPP_INFO(this->get_logger(), "LDLiDAR Node Started");

    // ==== 파라미터 설정 ====
    std::string port_name = "/dev/ttyUSB0";
    uint32_t baudrate = 230400;
    std::string frame_id = "base_laser";

    // 퍼블리셔 생성
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

    // 드라이버 객체 생성
    driver_ = std::make_unique<ldlidar::LDLidarDriver>();
    driver_->EnableFilterAlgorithnmProcess(true);

    // === 타임스탬프 콜백 등록 ===
    driver_->RegisterGetTimestampFunctional(std::bind(&LDLidarNode::GetTimestamp, this));

    // === LiDAR 시작 ===
    if (!driver_->Start(ldlidar::LDType::LD_14, port_name, baudrate)) {
      RCLCPP_ERROR(this->get_logger(), "LiDAR 연결 실패");
      rclcpp::shutdown();
      return;
    }

    // 타이머: 20Hz 주기
    timer_ = this->create_wall_timer(50ms, std::bind(&LDLidarNode::publishScan, this));
    frame_id_ = frame_id;
  }

private:
  // ===== LaserScan 데이터 발행 함수 =====
  void publishScan() {
    ldlidar::LaserScan scan_data;
    auto status = driver_->GetLaserScanData(scan_data, 100);

    if (status == ldlidar::LidarStatus::NORMAL) {
      auto msg = std::make_shared<sensor_msgs::msg::LaserScan>();
      msg->header.stamp = this->now();
      msg->header.frame_id = frame_id_;

      if (scan_data.points.empty()) return;

      float angle_min = scan_data.points.front().angle;
      float angle_max = scan_data.points.back().angle;
      size_t point_count = scan_data.points.size();

      msg->angle_min = angle_min * M_PI / 180.0;
      msg->angle_max = angle_max * M_PI / 180.0;
      msg->angle_increment = (msg->angle_max - msg->angle_min) / point_count;
      msg->time_increment = 0.0;
      msg->scan_time = 0.1;
      msg->range_min = 0.05;
      msg->range_max = 12.0;

      for (const auto& pt : scan_data.points) {
        msg->ranges.push_back(static_cast<float>(pt.distance) / 1000.0);
        msg->intensities.push_back(static_cast<float>(pt.intensity));
      }

      scan_pub_->publish(*msg);
    } else {
      RCLCPP_WARN(this->get_logger(), "Laser scan data 수신 실패 또는 비정상 상태");
    }
  }

  // ===== 타임스탬프 콜백 함수 =====
  uint64_t GetTimestamp() {
    rclcpp::Time now = this->get_clock()->now();
    return static_cast<uint64_t>(now.nanoseconds() / 1000);  // us 단위
  }

  std::unique_ptr<ldlidar::LDLidarDriver> driver_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string frame_id_;
};

// ===== 메인 함수 =====
int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LDLidarNode>());
  rclcpp::shutdown();
  return 0;
}
 
