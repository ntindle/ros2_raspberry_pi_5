#!/bin/bash
# ROS2 Jazzy Installation Script for Debian Bookworm (Tier 3 platform)

# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Install required packages
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS 2 GPG key
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/debian bookworm main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install development tools
sudo apt update && sudo apt install -y \
  python3-flake8-blind-except \
  python3-flake8-class-newline \
  python3-flake8-deprecated \
  python3-mypy \
  python3-pip \
  python3-pytest \
  python3-pytest-cov \
  python3-pytest-mock \
  python3-pytest-repeat \
  python3-pytest-rerunfailures \
  python3-pytest-runner \
  python3-pytest-timeout \
  ros-dev-tools

# Create workspace and get ROS 2 code
mkdir -p ~/ros2_jazzy/src
cd ~/ros2_jazzy
vcs import --input https://raw.githubusercontent.com/ros2/ros2/jazzy/ros2.repos src

# Update system and install dependencies
sudo apt upgrade -y

# Configure rosdep
sudo rosdep init || true  # Added || true to prevent script failure if already initialized
rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers  python3-vcstool"

# Add swap space for Raspberry Pi (4GB)
sudo fallocate -l 4G /swapfile || true
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

# Build ROS 2
cd ~/ros2_jazzy/
colcon build --symlink-install --parallel-workers $(nproc) --executor parallel

# Setup environment
echo "source ~/ros2_jazzy/install/local_setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "ROS2 Jazzy installation completed!"
