# PX4 + ROS2 Docker Setup for Jetson Nano
## Overview

This tutorial explains how to setup the jetson nano for FOXY

### Prerequisites
   * Install 20.04 ubuntu [Image](https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image)
   * [ROS2 Foxy Installed](https://docs.px4.io/main/en/ros/ros2_comm.html#install-ros-2),
   * [PX4-Autopilot downloaded](https://docs.px4.io/main/en/dev_setup/building_px4.html)
   * Python 3.8

## Install PX4 Offboard and dependencies (one time setup)

install:
```
sudo apt install python3-rosdep2
pip3 install --user kconfiglib
```

### Install the px4-offboard example

```
cd ~
mkdir paraceptor
git clone https://github.com/tassalor1/paraceptor.git
```

### Install PX4 msg

The `px4-offboard` example requires `px4_msgs` definitions:

```
mkdir -p ~/px4_ros_com_ws/src && cd ~/px4_ros_com_ws/src
git clone https://github.com/PX4/px4_msgs.git
```

Build:

```
colcon build
```

## Install the micro_ros_agent  (one time setup)
Follow these instructions to build the micro_ros_setup:  [Building micro_ros_setup](https://github.com/micro-ROS/micro_ros_setup#building)
```
source /opt/ros/foxy/setup.bash

mkdir microros_ws && cd microros_ws

git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

rosdep update && rosdep install --from-paths src --ignore-src -y

colcon build

source install/local_setup.bash
```

Follow these instructions to build the micro_ros_agent:  [Building micro-ROS-Agent](https://github.com/micro-ROS/micro_ros_setup#building-micro-ros-agent)
```
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.sh
```

## Run micro_ros_agent 
```
source ~/docker-build/px4_ros_com_ws/install/setup.bash
source ../px4_ros_com_ws/src/install/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export PYTHONOPTIMIZE=1
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 ROS_DOMAIN_ID=0
```

## Run offboard 
```
source /home/jetson/px4_ros_com_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export PYTHONOPTIMIZE=1
python3 px4_offboard/uav_detection_v2.py
```
5. Reduce unnecessary services:
```bash
sudo systemctl disable snapd.service
sudo systemctl disable ModemManager.service
```
