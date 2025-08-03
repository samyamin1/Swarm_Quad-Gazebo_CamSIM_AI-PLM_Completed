# Multi-stage build for M1 MacBook optimization
FROM --platform=linux/arm64 ubuntu:22.04 as base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV GAZEBO_VERSION=fortress

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

# Install ROS 2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-plugins \
    ros-humble-gazebo-dev \
    ros-humble-joy \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo Fortress
RUN curl -sSL https://packages.osrfoundation.org/gpg.key | apt-key add - \
    && echo "deb https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list

RUN apt-get update && apt-get install -y \
    gazebo \
    libgazebo-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install \
    numpy \
    scipy \
    matplotlib \
    pytest \
    pytest-ros \
    pyyaml \
    transforms3d

# Create workspace
WORKDIR /workspace

# Copy source code
COPY src/ ./src/
COPY launch/ ./launch/
COPY worlds/ ./worlds/
COPY models/ ./models/
COPY config/ ./config/
COPY scripts/ ./scripts/
COPY tests/ ./tests/

# Build ROS 2 workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install"

# Create entrypoint script
RUN echo '#!/bin/bash\n\
source /opt/ros/humble/setup.bash\n\
source /workspace/install/setup.bash\n\
exec "$@"' > /entrypoint.sh && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"] 