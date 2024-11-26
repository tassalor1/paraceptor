FROM dustynv/ros:humble-ros-base-l4t-r32.7.1

# Update and install dependencies
RUN apt update && apt install -y \
    python3-pip python3-venv \
    git colcon-common-extensions \
    qgroundcontrol cmake build-essential libssl-dev

# Install Python dependencies (e.g., for PX4 Offboard and YOLOv5)
RUN pip install torch torchvision

# Clone required repositories
WORKDIR /workspace
RUN git clone https://github.com/PX4/PX4-Autopilot.git
RUN git clone https://github.com/tassalor1/paraceptor.git paraceptor
RUN git clone https://github.com/PX4/px4_ros_com.git
RUN git clone https://github.com/PX4/px4_msgs.git

# Build ROS workspace for px4_msgs
WORKDIR /workspace/px4_ros_com_ws
RUN mkdir -p src && mv /workspace/px4_ros_com src/ && mv /workspace/px4_msgs src/
RUN colcon build

# Install Fast DDS
WORKDIR /workspace
RUN git clone --recursive https://github.com/eProsima/Fast-DDS.git -b v2.0.2 FastDDS-2.0.2
WORKDIR /workspace/FastDDS-2.0.2/build
RUN cmake -DTHIRDPARTY=ON -DSECURITY=ON .. && \
    make -j$(nproc --all) && \
    make install

# Install micro-ROS agent
WORKDIR /workspace/microros_ws
RUN git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
RUN rosdep update && rosdep install --from-paths src --ignore-src -y
RUN colcon build
RUN source install/local_setup.bash && \
    ros2 run micro_ros_setup create_agent_ws.sh && \
    ros2 run micro_ros_setup build_agent.sh

# Set up environment variables for ROS 2 and PX4
ENV ROS_DOMAIN_ID=0 \
    PYTHONOPTIMIZE=1

# Default working directory
WORKDIR /workspace
