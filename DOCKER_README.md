Prerequsiteis 
 * Run for GUI in docker `xhost +local:docker`

 * Run the docker image
 * cd into PX4 in docker and build it `docker exec -it blissful_tesla bash -c "cd /PX4-Autopilot && make px4_sitl_default"`

 * Run `docker ps` - you should see `Name`
 * Take this `Name` and replce with [container_name] in each command

# Run Docker Image
```
docker run -it --network host --privileged -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix px4_ros2_sim
```

# RUN MULTI DRONE
## Terminal 1 
```
docker exec -it [container_name] -c "source /opt/ros/humble/setup.bash && cd /microros_ws && source install/setup.bash && export ROS_DOMAIN_ID=0 && export PYTHONOPTIMIZE=1 && ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 ROS_DOMAIN_ID=0"
```
## Terminal 2 Bridge from home directory 
```
docker exec -it [container_name] bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && ros2 run ros_gz_image image_bridge /camera"
```

## Terminal 3 Drone 1 Recon
```
docker exec -it [container_name] bash -c "cd /PX4-Autopilot && export ROS_DOMAIN_ID=0 && export PYTHONOPTIMIZE=1 && export GZ_SIM_RESOURCE_PATH=/PX4-Autopilot/Tools/sitl_gazebo/models && PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE='7,0,0,0,0,0' PX4_SIM_MODEL=1040_gazebo-classic_standard_vtol ./build/px4_sitl_default/bin/px4 -i 1"
```

## Terminal 4 Drone 2 Inteceptor
```
docker exec -it [container_name] bash -c "cd /PX4-Autopilot && export ROS_DOMAIN_ID=0 && export PYTHONOPTIMIZE=1 && export GZ_SIM_RESOURCE_PATH=/PX4-Autopilot/Tools/sitl_gazebo/models && PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE='0,0,0,0,0,0' PX4_SIM_MODEL=4002_gz_x500_depth ./build/px4_sitl_default/bin/px4 -i 2"
```

## Terminal 5 
```
docker exec -it [container_name] bash -c "chmod +x /usr/local/bin/QGroundControl.AppImage && /usr/local/bin/QGroundControl.AppImage"
```

## Terminal 6 
```
docker exec -it [container_name] bash -c "cd /paraceptor && source /install/setup.bash && export ROS_DOMAIN_ID=0 && export PYTHONOPTIMIZE=1 && source /px4_ros_com_ws/install/setup.bash && python px4_offboard/uav_detection_v2.py"
```
You may have to replace the path to best_fixed.pt in line 27 of the uav_detection_v2.py, so that the path is appropriate to your local system. 
## Terminal 7
```
docker exec -it [container_name] bash -c "cd /paraceptor && source /install/setup.bash && export ROS_DOMAIN_ID=0 && export PYTHONOPTIMIZE=1 && source /px4_ros_com_ws/install/setup.bash && source /install/setup.bash && ros2 launch px4_offboard offboard_position_control.launch.py"
```