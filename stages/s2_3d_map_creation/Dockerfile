FROM ubuntu:22.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install system dependencies
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    wget \
    git \
    build-essential \
    cmake \
    python3 \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Add ROS 2 repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-rviz2 \
    ros-humble-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install \
    numpy \
    scipy \
    matplotlib \
    opencv-python \
    transforms3d

# Set up workspace
WORKDIR /workspace

# Copy Stage 1 quadcopter controller
COPY ../s1_quadcopter_simulation/src/quadcopter_controller /workspace/src/quadcopter_controller/

# Copy Stage 2 worlds and tests
COPY worlds /workspace/worlds/
COPY test_hospital_world.py /workspace/test_hospital_world.py
COPY launch /workspace/launch/

# Build the workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select quadcopter_controller

# Set up environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /workspace/install/setup.bash" >> ~/.bashrc

# Expose ports for Gazebo GUI
EXPOSE 11345

# Default command
CMD ["/bin/bash"] 