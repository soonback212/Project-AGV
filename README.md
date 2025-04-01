cd ~/agv_ws/src

# 1. Cartographer ROS 소스 클론
git clone https://github.com/cartographer-project/cartographer_ros.git

# 2. 의존성 설치 (첫 설치라면)
sudo apt install ros-humble-cartographer ros-humble-cartographer-ros-msgs

# 3. 워크스페이스 루트로 이동 후 빌드
cd ~/agv_ws
colcon build --packages-select cartographer_ros
source install/setup.bash


jdamr@jdamr-pc:~/agv_ws/src$ sudo apt install ros-humble-cartographer ros-humble-cartographer-ros-msgs
Reading package lists... Done
Building dependency tree... Done
Reading state information... Done
ros-humble-cartographer is already the newest version (2.0.9003-1jammy.20241125.211935).
ros-humble-cartographer-ros-msgs is already the newest version (2.0.9002-1jammy.20241128.021647).
ros-humble-cartographer-ros-msgs set to manually installed.
You might want to run 'apt --fix-broken install' to correct these.
The following packages have unmet dependencies:
 libsdformat9-dev : Depends: libsdformat9 (= 9.10.1-1~focal) but it is not going to be installed
E: Unmet dependencies. Try 'apt --fix-broken install' with no packages (or specify a solution).
