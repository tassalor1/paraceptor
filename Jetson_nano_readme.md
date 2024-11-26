# PX4 + ROS2 Docker Setup for Jetson Nano


## Pull and Run the Docker container
docker pull dustynv/ros:humble-ros-base-l4t-r32.7.1
docker run --rm -it --runtime nvidia --network host --gpus all -e DISPLAY dustynv/ros:humble-ros-base-l4t-r32.7.1

## Now Follow
### Prerequisites

   * [FastDDS Installed](https://docs.px4.io/v1.13/en/dev_setup/fast-dds-installation.html#fast-dds-installation)
   * git clone https://github.com/PX4/PX4-Autopilot.git

```
cd ~
mkdir paraceptor
git clone https://${GIT_TOKEN}@github.com/tassalor1/paraceptor.git paraceptor
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

This should build. You may see some warnings interspered with the output.  As long as there are no __*errors*__ you should be OK..

## Install the micro_ros_agent  (one time setup)
Follow these instructions to build the micro_ros_setup:  [Building micro_ros_setup](https://github.com/micro-ROS/micro_ros_setup#building)
```
source /opt/ros/humble/setup.bash

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
