# px4-offboard
## Prerequisites
   * [ROS2 Installed](https://docs.px4.io/main/en/ros/ros2_comm.html#install-ros-2), and setup for your operating system (e.g. [Linux Ubuntu](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html)) with Gazebo
   * [FastDDS Installed](https://docs.px4.io/v1.13/en/dev_setup/fast-dds-installation.html#fast-dds-installation)
   * [PX4-Autopilot downloaded](https://docs.px4.io/main/en/dev_setup/building_px4.html)
   * [QGroundControl installed](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html)
   * Ubuntu 22.04
   * ROS2 Humble
   * Python 3.10
   * Torch for the yolov5 model

Refer to doc for install


## For CV ros-gzgarden ##
```
sudo apt install ros-humble-ros-gzgarden
```

## RUN MULTI DRONE
## Terminal 1 
```
cd ~/microros_ws
source ../px4_ros_com_ws/src/install/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export PYTHONOPTIMIZE=1
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 ROS_DOMAIN_ID=0
```
## Terminal 2 Bridge from home directory 
```
source /opt/ros/humble/setup.bash
ros2 run ros_gz_image image_bridge /camera
```

## Terminal 3 Drone 1 Recon
```
cd ~/PX4-Autopilot
source ~/PX4-Autopilot/install/setup.bash
export PATH=$PATH:~/PX4-Autopilot/build/px4_sitl_default/bin
export ROS_DOMAIN_ID=0
export PYTHONOPTIMIZE=1
export GZ_SIM_RESOURCE_PATH=~/PX4-Autopilot/Tools/sitl_gazebo/models
PX4_GZ_WORLD=baylands 
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="7,0,0,0,0,0" PX4_SIM_MODEL=standard_vtol ./build/px4_sitl_default/bin/px4 -i 1
```

```
pkill -f "gz sim"
```

## Terminal 5 Drone 2 Inteceptor
```
cd ~/PX4-Autopilot
source ~/PX4-Autopilot/install/setup.bash
export ROS_DOMAIN_ID=0
export PYTHONOPTIMIZE=1
export GZ_SIM_RESOURCE_PATH=~/PX4-Autopilot/Tools/sitl_gazebo/models
PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL_POSE="0,0,0,0,0,0" \
PX4_GZ_WORLD=baylands \
PX4_SIM_MODEL=x500_mono_cam \
./build/px4_sitl_default/bin/px4 -i 2
```

## Terminal 5 from home directory 
```
source install/setup.bash
chmod +x ./QGroundControl.AppImage
./QGroundControl.AppImage 
```
Click Takeoff from left hand menu, then slide to confirm

## Terminal 6 
```
cd ~/paraceptor
source install/setup.bash
export ROS_DOMAIN_ID=0
export PYTHONOPTIMIZE=1
source ../px4_ros_com_ws/src/install/setup.bash
python3 px4_offboard/uav_detection_v2.py
```
You may have to replace the path to best_fixed.pt in line 27 of the uav_detection_v2.py, so that the path is appropriate to your local system. 
## Terminal 7
```
cd ~/paraceptor
source install/setup.bash
export ROS_DOMAIN_ID=0
export PYTHONOPTIMIZE=1
source ../px4_ros_com_ws/src/install/setup.bash
source install/setup.bash
ros2 launch px4_offboard cv_offboard.launch.py sim:=true
```
## After script changes #####################
```
colcon build --packages-select px4_offboard
source install/setup.bash
ros2 launch px4_offboard offboard_position_control.launch.py
```
### Base Station Integration Test

The base station is a mockup radar system which constantly publishes the enemy drone's predicted position 5 seconds into the future. So you just have to subscribe to the base station and use that as the trajectory setpoint. 

cd into your colcon workspace, and replace the paraceptor package with this repo. Go the the root of the repository, then:
```
colcon build --packages-select px4_offboard
source install/setup.bash
```
Start the recon drone, the interceptor drone and qground control. Then run

```
ros2 launch px4_offboard offboard_position_control.launch.py
```

To verify how the system works, run

```
ros2 run rqt_graph rqt_graph
```

*NOTE*: Switch to offboard control after manually doing takeoff. Currently automatic take-off is a bit buggy.

*TODO*: Height not more than 3 metres. Fix that.



