# 🚗 Project-AGV: Stage 1 - Teleop 기반 AGV 제어

> **목표:**  
초기 AGV 플랫폼에서 아두이노 기반 모터 제어와 ROS2 Teleop 키보드 입력을 통해 **수동 제어(teleoperation)**가 가능하도록 구성합니다.  
향후 SLAM, 딥러닝 제어, 로봇 암 확장으로 이어지는 **기초 제어 기반 구축 단계**입니다.

---

## 🧩 Stage1 구조 개요

```
Project-AGV/
└── stage1_teleop/
    ├── motor_serial/                  # ROS2 패키지
    │   ├── motor_serial_node.py       # 아두이노와 시리얼 통신 노드
    │   ├── __init__.py
    ├── launch/
    │   └── stage1_launch.py           # 통합 실행 launch 파일
    ├── arduino/
    │   └── stage1_motor.ino           # 아두이노 Mega2560용 모터 제어 코드
    ├── package.xml
    └── setup.py
```

---

## 🔧 사용 기술 및 환경

- **ROS2**: Humble (Ubuntu 22.04)
- **아두이노**: Mega2560, 엔코더 내장 DC 모터
- **모터 드라이버 보드**: TB6612 기반
- **통신 방식**: UART 시리얼 통신 (115200bps)
- **제어 대상**: 전/후진 및 좌/우회전 가능한 2개 모터

---

## 🧠 주요 구성 요소

### 1. `motor_serial_node.py`
- `/cmd_vel` 토픽 구독 (geometry_msgs/Twist)
- Twist 메시지를 문자열로 변환하여 시리얼로 전송
  - 예: `"L100 R100\n"`, `"L-80 R80\n"`

### 2. `stage1_motor.ino`
- 아두이노에서 시리얼 수신 후 PWM + DIR 핀 제어
- TB6612 드라이버 기반으로 모터 동작 수행

### 3. `stage1_launch.py`
- `motor_serial_node` 실행
- `teleop_twist_keyboard` 키보드 입력 노드 병렬 실행

---

## 🧪 실행 방법

### 1. 아두이노 업로드
- `/arduino/stage1_motor.ino`를 Arduino IDE로 Mega2560에 업로드

### 2. ROS2 빌드 및 실행
```bash
cd ~/agv_ws/
colcon build --packages-select motor_serial
source install/setup.bash
ros2 launch motor_serial stage1_launch.py
```

### 3. Teleop 키보드로 AGV 제어
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

- `w/s`: 전진 / 후진  
- `a/d`: 좌회전 / 우회전  
- `x`: 정지

---

## 🧰 통신 구조도

```
┌──────────────┐       UART        ┌──────────────────────┐
│ teleop_twist │ ─────────────▶    │ motor_serial_node.py │
│  (keyboard)  │     /cmd_vel      └────────┬─────────────┘
└──────────────┘                            ▼
                                      Serial 문자열 전송
                                            ▼
                                   ┌────────────────────┐
                                   │ Arduino Mega2560   │
                                   │ + TB6612 + 모터 제어 │
                                   └────────────────────┘
```

---

## ⏭️ 다음 단계 (Stage2 예고)

- IMU 센서 연동 (MPU6500)
- LiDAR 기반 SLAM 구성
- 로봇 암 기초 프레임 확장
- 자율 주행을 위한 딥러닝 명령 인식 노드 개발

---

> 이 프로젝트는 가정용 스마트 도우미 로봇의 자율 주행 기반을 위한 초기 단계입니다.  
> 꾸준한 단계별 개발로 삼성봇 핸디 유사 시스템 구현을 목표로 합니다.

