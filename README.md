# px4-offboard
## Prerequisites
   * [ROS2 Installed](https://docs.px4.io/main/en/ros/ros2_comm.html#install-ros-2), and setup for your operating system (e.g. [Linux Ubuntu](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html)) with Gazebo
   * [FastDDS Installed](https://docs.px4.io/v1.13/en/dev_setup/fast-dds-installation.html#fast-dds-installation)
   * [PX4-Autopilot downloaded](https://docs.px4.io/main/en/dev_setup/building_px4.html)
   * [QGroundControl installed](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html)
   * Ubuntu 22.04
   * ROS2 Humble
   * Python 3.10

Refer to doc for install

## RUN SINGLE DRONE
## Terminal 1 
```
cd ~/microros_ws
source ../px4_ros_com_ws/install/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export PYTHONOPTIMIZE=1
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 ROS_DOMAIN_ID=0
```

## Terminal 2 
```
cd ~/PX4-Autopilot
source install/setup.bash
export ROS_DOMAIN_ID=0
export PYTHONOPTIMIZE=1
make px4_sitl gz_x500
```

## Terminal 3 from home directory 
```
source install/setup.bash
chmod +x ./QGroundControl.AppImage
./QGroundControl.AppImage 
```
Click Takeoff from left hand menu, then slide to confirm

## Terminal 4 
```
cd ~/px4-offboard
source install/setup.bash
export ROS_DOMAIN_ID=0
export PYTHONOPTIMIZE=1
ros2 topic list
source ../px4_ros_com_ws/install/setup.bash
source install/setup.bash
```

```
ros2 launch px4_offboard offboard_position_control.launch.py
```

Now head back to QGroundControl and enable offboard control.  
Click Takeoff from left hand menu, then slide to confirm
Click the current mode "HOLD" in upper left, then in the menu, select "Offboard":

After a 1-2 sec pause, the demo should take control and you should see the 3d indicator in Rviz drawing circles.


## After script changes #####################
```
colcon build --packages-select px4_offboard
source install/setup.bash
ros2 launch px4_offboard offboard_position_control.launch.py
```

## RUN MULTI DRONE
## Terminal 1 
```
cd ~/microros_ws
source ../px4_ros_com_ws/install/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export PYTHONOPTIMIZE=1
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 ROS_DOMAIN_ID=0
```

## Terminal 2 Drone 1 Recon
```
cd ~/PX4-Autopilot
source install/setup.bash
export ROS_DOMAIN_ID=0
export PYTHONOPTIMIZE=1
PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=standard_vtol ./build/px4_sitl_default/bin/px4 -i 1
```

## Terminal 3 Drone 2 Inteceptor
```
cd ~/PX4-Autopilot
source install/setup.bash
export ROS_DOMAIN_ID=0
export PYTHONOPTIMIZE=1
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,1" PX4_SIM_MODEL=px4vision ./build/px4_sitl_default/bin/px4 -i 2
```

## Terminal 4 from home directory 
```
source install/setup.bash
chmod +x ./QGroundControl.AppImage
./QGroundControl.AppImage 
```
Click Takeoff from left hand menu, then slide to confirm

## Terminal 5
```
cd ~/px4-offboard
source install/setup.bash
export ROS_DOMAIN_ID=0
export PYTHONOPTIMIZE=1
source ../px4_ros_com_ws/install/setup.bash
source install/setup.bash
ros2 launch px4_offboard offboard_position_control.launch.py
```



### Hardware

This section is intended for running the offboard control node on a companion computer, such as a Raspberry Pi or Nvidia Jetson/Xavier. You will either need an SSH connection to run this node, or have a shell script to run the nodes on start up. 

If you are running this through a UART connection into the USB port, start the micro-ros agent with the following command

```
micro-ros-agent serial --dev /dev/ttyUSB0 -b 921600 -v
```
If you are using a UART connection which goes into the pinouts on the board, start the micro-ros agent with the following comand
```
micro-ros-agent serial --dev /dev/ttyTHS1 -b 921600 -V
```

To run the offboard position control example, run the node on the companion computer
```
ros2 launch px4_offboard offboard_hardware_position_control.launch.py
```
