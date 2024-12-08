# Use NVIDIA's ROS2 Humble base image for Jetson
FROM dustynv/ros:humble-ros-base-l4t-r32.7.1

# Set environment variables
ENV ROS_DISTRO humble
ENV ROS_OS_OVERRIDE=ubuntu:jammy

# Install Python 3.8
RUN apt-get update && apt-get install -y software-properties-common \
    && add-apt-repository ppa:deadsnakes/ppa \
    && apt-get update && apt-get install -y \
    python3.8 python3.8-dev python3.8-distutils python3.8-venv \
    && rm -rf /var/lib/apt/lists/*

# Set Python 3.8 as default
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.6 1 \
    && update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 2 \
    && update-alternatives --set python3 /usr/bin/python3.8

# Install pip for Python 3.8
RUN wget https://bootstrap.pypa.io/get-pip.py \
    && python3 get-pip.py \
    && rm get-pip.py

# Set the working directory to /root
# Use NVIDIA's ROS2 Humble base image for Jetson
FROM dustynv/ros:humble-ros-base-l4t-r32.7.1

# Set environment variables
ENV ROS_DISTRO humble

# Set the working directory to /root/ros_ws
WORKDIR /root/ros_ws
RUN mkdir src

# Install necessary system dependencies
RUN apt-get update && apt-get install -y \
    nano \
    git \
    wget \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install necessary Python packages
RUN pip3 install \
    kconfiglib \
    empy \
    toml \
    jinja2 \
    pyserial \
    numpy \
    cerberus \
    pyros-genmsg \
    packaging

# Install Foonathan Memory
RUN git clone https://github.com/eProsima/foonathan_memory_vendor.git /tmp/foonathan_memory_vendor && \
    cd /tmp/foonathan_memory_vendor && \
    mkdir build && cd build && \
    cmake .. && \
    cmake --build . --target install && \
    rm -rf /tmp/foonathan_memory_vendor

# Install Fast DDS
RUN git clone --recursive https://github.com/eProsima/Fast-DDS.git -b v2.0.2 /tmp/Fast-DDS && \
    cd /tmp/Fast-DDS && \
    mkdir build && cd build && \
    cmake -DTHIRDPARTY=ON -DSECURITY=ON -DCMAKE_INSTALL_PREFIX=/usr/local .. && \
    make -j$(nproc) && \
    make install && \
    rm -rf /tmp/Fast-DDS

# Copy your application code into the container
COPY ./paraceptor src/paraceptor

# Clone PX4 Autopilot into src directory
RUN git clone https://github.com/PX4/PX4-Autopilot.git src/PX4-Autopilot

# Clone missing dependencies into src
RUN git clone https://github.com/ament/ament_cmake.git src/ament_cmake
RUN git clone https://github.com/ament/ament_lint.git src/ament_lint

# Install ROS2 packages from source using the script
ADD https://raw.githubusercontent.com/dusty-nv/jetson-containers/master/packages/ros/ros2_install.sh /usr/local/bin/ros2_install.sh
RUN chmod +x /usr/local/bin/ros2_install.sh
RUN /usr/local/bin/ros2_install.sh https://github.com/PX4/px4_msgs.git
RUN /usr/local/bin/ros2_install.sh https://github.com/micro-ROS/micro_ros_setup.git -b $ROS_DISTRO

# Source the ROS environment and build your workspace
RUN /bin/bash -c "source /ros_entrypoint.sh && \
    cd /root/ros_ws && \
    colcon build"


# Set the default command to bash
CMD ["/bin/bash"]
