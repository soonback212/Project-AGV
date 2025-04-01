sudo rm /var/cache/apt/archives/libsdformat9_9.10.1-1~focal_arm64.deb

sudo apt clean
sudo apt update
sudo apt --fix-broken install

sudo apt install libsdformat9

sudo apt install ros-humble-cartographer ros-humble-cartographer-ros
