# PX4 + ROS2 Docker Setup for Jetson Nano


## Build and Run the Docker Image
From the directory containing the `Dockerfile`, build the Docker image:
```
docker build -t px4-ros2-drone .
```
Start the container with the following command:
```
docker run --runtime nvidia --network host --gpus all --name drone-container \
    -v /path/to/data:/workspace/data \
    fastdds-ros2-drone
```
