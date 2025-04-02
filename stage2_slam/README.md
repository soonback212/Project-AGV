# AGV SLAM 시스템 구성 및 실행 가이드

이 프로젝트는 ROS 2 기반 AGV(자율 주행 차량)를 위한 **SLAM 시스템**입니다.
LDLiDAR LD14, IMU, 엔코더 기반 오도메트리, 그리고 Cartographer를 활용해 실시간 지도를 작성하고 로봇의 위치를 추정합니다.

---

## 📁 프로젝트 구성

```
Project-AGV-main/
├── stage2_slam/
│   ├── cartographer_agv/         # SLAM 관련 설정 및 실행 코드
│   │   ├── launch/
│   │   │   ├── agv_launch.py
│   │   │   └── occupancy_grid.launch.py
│   │   ├── config/
│   │   │   └── agv.lua
│   │   ├── urdf/
│   │   │   └── agv.urdf.xacro
│   │   └── rviz/
│   │       └── agv_cartographer.rviz
│   ├── slam_control/             # IMU, 엔코더 처리 노드 (imu_parser_node, motor_serial_node)
```

---

## ⚙️ 사전 설치 요구사항

### ROS 2 Humble + Cartographer
```bash
sudo apt update
sudo apt install ros-humble-cartographer ros-humble-cartographer-ros-msgs
```

### 의존 패키지
```bash
sudo apt install ros-humble-tf2-ros ros-humble-robot-state-publisher ros-humble-sensor-msgs ros-humble-nav-msgs
```

---

## 🛠️ 빌드 방법
```bash
cd ~/agv_ws
colcon build --packages-select cartographer_agv slam_control
source install/setup.bash
```

> **Note**: 설치 후 `.bashrc`에 `source ~/agv_ws/install/setup.bash` 추가 추천

---

## 🚀 실행 방법
```bash
ros2 launch cartographer_agv agv_launch.py
```

실행 시 다음 구성 요소들이 자동 실행됩니다:
- LD14 LiDAR 드라이버 (`ldlidar_sl_ros2`)
- `robot_state_publisher` (URDF → TF 변환)
- Cartographer SLAM 노드
- Occupancy Grid 맵 퍼블리셔
- IMU 노드 (`imu_parser_node`)
- 엔코더 노드 (`motor_serial_node`)
- RViz2 (5초 지연 실행)

---

## 📌 핵심 기능 설명

### 1. IMU + 엔코더 기반 Odometry (`slam_control/imu_parser_node`)
- 초기 보정(Bias 제거)
- 자이로 드리프트 보완을 위한 **Complementary Filter** 적용
- 정지 상태에서는 드리프트 누적 방지 로직 내장

### 2. agv.urdf.xacro
- 로봇 모델 정의
- LiDAR 및 IMU 센서의 정확한 위치 포함 (예: LiDAR 위치 `z=0.18`, IMU 정중앙 등)

### 3. Cartographer 설정 (`agv.lua`)
- LiDAR topic: `/scan`
- IMU topic: `/imu`
- Odometry topic: `/odom`
- Frame 설정: `map → odom → base_link`

---

## 📍 단계별 목표

### ✅ Step 2: SLAM 및 네비게이션 구현
- 실시간 맵 작성(Cartographer)
- Occupancy Grid 저장 (`ros2 run nav2_map_server map_saver_cli`)
- 저장된 맵을 이용한 네비게이션 구현 (Step 3 이전까지의 목표)

### 🔄 Step 3: 딥러닝 기반 행동 판단 (Vision 기반)
- 카메라를 통한 객체 인식 (YOLO 등 활용)
- 객체 기반 행동 결정, 추적, 회피 등 고차원 행동 구현
- ROS2 bridge를 통한 학습 모델 연동

---

## 🖥️ RViz 설정 (agv_cartographer.rviz)
- LaserScan, TF, RobotModel, Odometry, Map 시각화 포함
- `Use Simulation Time`은 `false`로 설정됨 (실센서 기준)

---

## ✅ 트러블슈팅

| 문제 | 해결 방법 |
|------|-------------|
| `.urdf.xacro not found` | `agv.urdf` → `agv.urdf.xacro`로 이름 변경하거나, launch 파일에서 경로 수동 지정 |
| SLAM이 실행되지 않음 | `agv.lua`, `/scan`, `/imu`, `/odom` 토픽명이 정확한지 확인 |
| RViz에 로봇 안 보임 | `robot_state_publisher`가 실행 중인지 확인 |
| 실행 시 Cartographer 에러 발생 | `ros-humble-cartographer`, `cartographer_ros`가 제대로 설치되었는지 확인 |
| RViz가 터미널에서 자동 실행 안 됨 | `rviz_config_file` 경로가 올바른지, 파일 존재 여부 확인 |
| IMU 값이 비정상적으로 드리프트 | IMU 노드 보정 상태 확인 (`calibrated == True`인지 확인 필요) |
| `occupancy_grid.launch.py` 에러 | 해당 파일 경로 확인 또는 Cartographer 설치 누락 여부 점검 |
| `/dev/ttyACM0` 포트 오류 | 포트 이름 확인 (`ls /dev/ttyACM*` 등) 후 launch 파일 수정 |

---

## 👨‍💻 작성자 & 테스트 환경
- **작성자**: 한지원 (Jiwon Han)
- **테스트 환경**: Ubuntu 22.04 + ROS 2 Humble + LD14 + Custom IMU + Encoder

---

> 마지막 업데이트: 2025.04.02
