FROM ubuntu:22.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV PYTHONOPTIMIZE=1
ENV ROS_DOMAIN_ID=0

# Install basic dependencies
RUN apt-get update && apt-get install -y \
    git \
    wget \
    curl \
    python3-pip \
    python3-venv \
    lsb-release \
    gnupg \
    software-properties-common

# Install ROS2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update && apt-get install -y ros-humble-desktop

# Install colcon and rosdep
RUN apt-get update && apt-get install -y python3-colcon-common-extensions python3-rosdep

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# Verify ROS2 and colcon installation
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && ros2 topic list"
RUN which colcon
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && ros2 pkg list"
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && ros2 node list"

# Install additional ROS packages
RUN apt-get update && apt-get install -y \
    ros-humble-ackermann-msgs \
    ros-humble-nav-msgs \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-image

# Install PX4 dependencies
RUN apt-get install -y \
    ninja-build \
    cmake \
    build-essential \
    genromfs \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    protobuf-compiler \
    libeigen3-dev

# Clone PX4-Autopilot
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive
RUN bash PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx

# Build PX4 SITL 
RUN cd PX4-Autopilot && make px4_sitl_default

# Install QGroundControl dependencies
RUN apt-get install -y \
    libsdl2-dev \
    libxcb-xinerama0 \
    libxcb-cursor0

# Download QGroundControl AppImage
RUN wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage -O /usr/local/bin/QGroundControl.AppImage
RUN chmod +x /usr/local/bin/QGroundControl.AppImage

# Setup Python virtual environment
RUN python3 -m venv /paraceptor_env

# Clone paraceptor repository
RUN mkdir -p /root/.ssh && chmod 700 /root/.ssh
COPY id_rsa /root/.ssh/id_rsa
RUN chmod 600 /root/.ssh/id_rsa
RUN ssh-keyscan github.com >> /root/.ssh/known_hosts
RUN git clone git@github.com:tassalor1/paraceptor.git

# Install Python dependencies
RUN /bin/bash -c "source /paraceptor_env/bin/activate && pip install --upgrade pip && pip install -r paraceptor/requirements.txt || true"

# Setup workspace
RUN mkdir -p /px4_ros_com_ws/src
WORKDIR /px4_ros_com_ws/src
RUN git clone https://github.com/PX4/px4_msgs.git
WORKDIR /px4_ros_com_ws
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build"

# Setup micro-ROS
RUN mkdir -p /microros_ws/src
WORKDIR /microros_ws/src
RUN git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git
WORKDIR /microros_ws
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && rosdep install --from-paths src --ignore-src -y && colcon build"
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
    source install/local_setup.bash && \
    ros2 run micro_ros_setup create_agent_ws.sh && \
    ros2 run micro_ros_setup build_agent.sh"

# Set working directory
WORKDIR /

# Add setup to bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source /px4_ros_com_ws/install/setup.bash" >> ~/.bashrc
RUN echo "source /microros_ws/install/setup.bash" >> ~/.bashrc
RUN echo "source /paraceptor_env/bin/activate" >> ~/.bashrc

# Set entrypoint
ENTRYPOINT ["/bin/bash"]
