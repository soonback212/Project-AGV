# Project-AGV: ROS2 기반 자율주행 AGV 시스템

**Project-AGV**는 ROS2를 기반으로 한 단계별 자율주행 AGV(Autonomous Guided Vehicle) 프로젝트입니다.
이 프로젝트는 실제 로봇 플랫폼(MyAGV)을 활용하여 **기초 제어부터 SLAM, 딥러닝 기반 비전 제어까지** 자율주행의 전 과정을 학습하고 구현하는 데 목적이 있습니다.

---

## 📅 프로젝트 개요

- **플랫폼**: ROS2 Humble + Ubuntu 22.04
- **하드웨어**: MyAGV, LD14 LiDAR, IMU, USB 카메라
- **프로그래밍 언어**: Python / C++ 혼합
- **구현 단계**:
  - Stage 1: 수동 제어 (Teleoperation)
  - Stage 2: SLAM 기반 자율주행
  - Stage 3: 카메라 + 딥러닝 기반 제어

---

## 🔄 프로젝트 목적

- 자율주행 로봇의 핵심 기술을 **이론이 아닌 직접 구현**을 통해 학습
- **ROS2의 구조, 노드 통신, 센서 통합, 좌표계, 맵핑, TF 시스템** 전반을 단계별로 실습
- 하드웨어 제어부터 인공지능 기반 이동까지 자율주행 전체 파이프라인을 이해

---

## 📌 각 Stage 요약

### ✅ Stage 1 - Teleoperation
- 조이스틱 또는 키보드로 수동 제어
- `/cmd_vel` 토픽으로 모터 제어 노드 동작 확인
- RViz에서 위치 및 센서 시각화 기초 연습
- [Stage 1 README 보기](https://github.com/soonback212/Project-AGV/blob/main/Stage1_teleop/README.md)

### ✅ Stage 2 - SLAM 기반 자율주행
- LiDAR + IMU + Encoder 조합으로 오도메트리 생성
- Cartographer를 활용한 실시간 맵 생성 (`/map`)
- TF 트리 구성: `map → odom → base_link`
- RViz를 통해 경로 및 센서 정보 시각화
- [Stage 2 README 보기](https://github.com/soonback212/Project-AGV/blob/main/stage2_slam/README.md)

### ✅ Stage 3 - 딥러닝 기반 비전 제어
- USB 카메라로 영상 입력 → 객체 인식 수행
- YOLO, OpenCV, TensorRT 기반 추론 모델 연결
- 인식 결과에 따라 `/cmd_vel`로 이동 명령
- 사람 따라가기, 특정 마커 인식 후 이동 등 고도화
- [Stage 3 README 보기 (작성 예정)]([./stage3_vision_control/README.md](https://github.com/soonback212/Project-AGV/blob/main/Stage1_teleop/README.md))

---

## 🤝 이 프로젝트를 통해 할 수 있는 것

- ROS2를 활용한 실제 로봇 제어 및 자율주행 시스템 개발 경험
- SLAM의 원리와 센서 퓨전 오도메트리 계산 로직 이해 및 구현
- 센서 데이터(`/scan`, `/imu`, `/odom`)를 바탕으로 맵핑 및 위치추정 가능
- 딥러닝 기반 컴퓨터 비전 제어까지 연계하여 **완전한 자율이동 로봇 구축 가능**

---

## ✨ 이런 사람에게 추천합니다

- ROS2 기반 로봇 개발 포트폴리오가 필요한 사람
- SLAM이나 Navigation을 직접 실습하고 싶은 사람
- 딥러닝 + 자율주행을 통합한 프로젝트 경험을 원하는 사람

---

## 🌟 향후 계획

- Stage 3 딥러닝 제어 정교화
- Navigation2 연동 및 목표지점 자동 주행 기능 확장
- 다중 로봇 시스템 통합
- 로봇 암과 연동하여 작업 자동화 기능 개발

---

## 📖 문서 정리

- [Stage 1 README 보기]([./stage1_teleop/README.md](https://github.com/soonback212/Project-AGV/blob/main/Stage1_teleop/README.md))
- [Stage 2 README 보기]([./stage2_slam/README.md](https://github.com/soonback212/Project-AGV/blob/main/stage2_slam/README.md))
- [Stage 3 README 보기 (작성 예정)](./stage3_vision_control/README.md)

---

## 👩‍💻 제작자

- 한지원 (AGV 제어 및 구조 설계)

