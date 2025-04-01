cd ~/agv_ws/src

# 1. Cartographer ROS 소스 클론
git clone https://github.com/cartographer-project/cartographer_ros.git

# 2. 의존성 설치 (첫 설치라면)
sudo apt install ros-humble-cartographer ros-humble-cartographer-ros-msgs

# 3. 워크스페이스 루트로 이동 후 빌드
cd ~/agv_ws
colcon build --packages-select cartographer_ros
source install/setup.bash
