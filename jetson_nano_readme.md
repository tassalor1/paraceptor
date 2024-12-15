## Overview

This tutorial explains how to setup the jetson nano for FOXY/ Arducam IMX477 and Pixracer FC

### Prerequisites
   * Install 20.04 ubuntu [Image](https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image)
   * [ROS2 Foxy Installed](https://docs.px4.io/main/en/ros/ros2_comm.html#install-ros-2),
   * [PX4-Autopilot downloaded](https://docs.px4.io/main/en/dev_setup/building_px4.html)
   * Python 3.8

## Install PX4 Offboard and dependencies (one time setup)

install:
```
sudo apt install python3-numpy
sudo apt install libboost-python1.71-dev libboost-dev
sudo apt install python3-rosdep2
sudo apt install ros-foxy-cv-bridge
pip3 install --user kconfiglib numpy==1.19.2 simple_pid timm
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
##  Camera settings
check jetpack for 4.6
Run the Jetson-IO Tool:
```
sudo /opt/nvidia/jetson-io/jetson-io.py
```
Configure the CSI Connector:

    Select "Configure Jetson Nano CSI Connector."
    "Configure for compatible hardware."
    Choose "Camera IMX477 Dual."
    Save pin changes and reboot when prompted.

check cv2 is using gstreamer
```
python3 -c "import cv2; print(cv2.getBuildInformation())"
```


for system stats
```
jtop
```

YOLO on Nano
Follow this [repo](https://github.com/mailrocketsystems/JetsonYolov5) add relevant files to new repo

Set CUDA paths:
```
export PATH=/usr/local/cuda-10.2/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda-10.2/lib64:$LD_LIBRARY_PATH
```
Install pycuda:
```
python3 -m pip install pycuda --user
```
Install torch & torchvision:
```
wget https://nvidia.box.com/shared/static/fjtbno0vpo676a25cgvuqc1wty0fkkg6.whl -O torch-1.10.0-cp36-cp36m-linux_aarch64.whl
pip3 install torch-1.10.0-cp36-cp36m-linux_aarch64.whl

git clone --branch v0.11.1 https://github.com/pytorch/vision torchvision
cd torchvision
sudo python3 setup.py install 
sudo python3 -m pip install -U jetson-stats==3.1.4
```
Maximize Nano performance:
```
sudo nvpmodel -m 0
sudo jetson_clocks
```
Version mismatch will occurs, use:

navargus some times will need restatrting if camera wont load, use:
```
sudo systemctl restart nvargus-daemon
```

```
sudo systemctl restart nvargus-daemon
source ~/px4_ros_com_ws/install/setup.bash
LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1 python3 px4_offboard/uav_detection_nano.py
```

## Connect Nano to pixracer

Remove ModemManager and set serial permissions:
```
sudo apt-get remove modemmanager -y
sudo chown root:dialout /dev/ttyTHS1
sudo chmod 660 /dev/ttyTHS1
```
Install pip and MAVProxy:
```
sudo apt-get install python3-pip -y
pip3 install MAVProxy
```
Run MAVProxy: This should say its connected
```
sudo mavproxy.py --master=/dev/ttyTHS1 --baudrate 57600 --aircraft my_drone
```
