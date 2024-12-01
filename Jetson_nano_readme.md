# PX4 + ROS2 Docker Setup for Jetson Nano


## Pull and Run the Docker container
```
docker pull dustynv/ros:humble-ros-base-l4t-r32.7.1
docker run -it --runtime nvidia --network host --gpus all -e DISPLAY dustynv/ros:humble-ros-base-l4t-r32.7.1
```

## Create `ros2_install` file
Inside the Docker container, create the `ros2_install.sh` script:
```
nano /usr/local/bin/ros2_install.sh
```
Copy and paste the contents from the [ros_install.sh](https://github.com/dusty-nv/jetson-containers/blob/master/packages/ros/ros2_install.sh) file into the script. 
Make the script executable:
```
sudo chmod +x /usr/local/bin/ros2_install.sh
```
## Build All ROS2 Packages from Source
# Run the following commands to build ROS2 packages from source:
```
/usr/local/bin/ros2_install.sh

# Example: Install jetson-inference nodes under /ros2_workspace
ROS_WORKSPACE=/ros2_workspace /usr/local/bin/ros2_install.sh \
    https://github.com/dusty-nv/ros_deep_learning
```
### Prerequisites

   * [FastDDS Installed](https://docs.px4.io/v1.13/en/dev_setup/fast-dds-installation.html#fast-dds-installation)
   * git clone https://github.com/PX4/PX4-Autopilot.git
    ```
    git clone https://github.com/PX4/PX4-Autopilot.git
    ```

```
cd ~
mkdir paraceptor
git clone https://${GIT_TOKEN}@github.com/tassalor1/paraceptor.git paraceptor
```

### Install PX4 msg

The `px4-offboard` example requires `px4_msgs` definitions:

```
mkdir -p ~/px4_ros_com_ws/src && cd ~/px4_ros_com_ws/src
/usr/local/bin/ros2_install.sh https://github.com/PX4/px4_msgs.git
```

Build:

```
colcon build
```

This should build. You may see some warnings interspered with the output.  As long as there are no __*errors*__ you should be OK..

## Install the micro_ros_agent  (one time setup)
Follow these instructions to build the micro_ros_setup:  [Building micro_ros_setup](https://github.com/micro-ROS/micro_ros_setup#building)
```
export ROS_DISTRO=humble

mkdir microros_ws && cd microros_ws
/usr/local/bin/ros2_install.sh https://github.com/micro-ROS/micro_ros_setup.git -b $ROS_DISTRO

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
