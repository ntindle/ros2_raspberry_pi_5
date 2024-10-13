#!/bin/bash
# ROS2 Humble Installation Script

# Install necessary packages
sudo apt update
sudo apt install -y git colcon python3-rosdep2 vcstool wget \
python3-flake8-docstrings python3-pip python3-pytest-cov \
python3-flake8-blind-except python3-flake8-builtins \
python3-flake8-class-newline python3-flake8-comprehensions \
python3-flake8-deprecated python3-flake8-import-order \
python3-flake8-quotes python3-pytest-repeat python3-pytest-rerunfailures \
python3-vcstools libx11-dev libxrandr-dev libasio-dev libtinyxml2-dev

# Create a new ROS2 workspace
mkdir -p ~/ros2_humble/src
cd ~/ros2_humble

# Download ROS2 source code
vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src

# Update system packages
sudo apt upgrade -y

# Configure rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers python3-vcstool"

# Build ROS2
colcon build --symlink-install

# Set up environment variables
echo "source ~/ros2_humble/install/local_setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "ROS2 Humble installation completed!"
