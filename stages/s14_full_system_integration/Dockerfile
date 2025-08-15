FROM ros:humble-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV PYTHONPATH=/opt/ros/humble/lib/python3/dist-packages:$PYTHONPATH

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-opencv \
    python3-numpy \
    python3-scipy \
    python3-matplotlib \
    python3-psutil \
    git \
    wget \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install \
    opencv-python \
    numpy \
    scipy \
    matplotlib \
    psutil \
    requests \
    websockets \
    flask \
    flask-socketio

# Initialize rosdep
RUN rosdep init && rosdep update

# Create workspace
WORKDIR /opt/ros/workspace

# Copy all stage source code
COPY ../s1_quadcopter_simulation/src/ ./src/
COPY ../s2_3d_map_creation/src/ ./src/
COPY ../s3_sensor_simulation/src/ ./src/
COPY ../s4_slam_engine/src/ ./src/
COPY ../s5_perception_plm/src/ ./src/
COPY ../s6_llm_decision_engine/src/ ./src/
COPY ../s7_navigation_control_loop/src/ ./src/
COPY ../s8_swarm_coordination/src/ ./src/
COPY ../s9_multi_sensor_fusion/src/ ./src/
COPY ../s10_advanced_slam/src/ ./src/
COPY ../s11_realtime_ai_integration/src/ ./src/
COPY ../s12_mission_planning_execution/src/ ./src/
COPY ../s13_performance_optimization/src/ ./src/
COPY ../s14_full_system_integration/src/ ./src/

# Copy launch files and configurations
COPY ../s1_quadcopter_simulation/launch/ ./launch/
COPY ../s2_3d_map_creation/launch/ ./launch/
COPY ../s3_sensor_simulation/launch/ ./launch/
COPY ../s4_slam_engine/launch/ ./launch/
COPY ../s5_perception_plm/launch/ ./launch/
COPY ../s6_llm_decision_engine/launch/ ./launch/
COPY ../s7_navigation_control_loop/launch/ ./launch/
COPY ../s8_swarm_coordination/launch/ ./launch/
COPY ../s9_multi_sensor_fusion/launch/ ./launch/
COPY ../s10_advanced_slam/launch/ ./launch/
COPY ../s11_realtime_ai_integration/launch/ ./launch/
COPY ../s12_mission_planning_execution/launch/ ./launch/
COPY ../s13_performance_optimization/launch/ ./launch/
COPY ../s14_full_system_integration/launch/ ./launch/

# Copy world files and models
COPY ../s1_quadcopter_simulation/models/ ./models/
COPY ../s1_quadcopter_simulation/worlds/ ./worlds/
COPY ../s2_3d_map_creation/worlds/ ./worlds/

# Build the workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Source the workspace
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /opt/ros/workspace/install/setup.bash" >> ~/.bashrc

# Create startup script
RUN echo '#!/bin/bash' > /startup.sh && \
    echo 'source /opt/ros/humble/setup.bash' >> /startup.sh && \
    echo 'source /opt/ros/workspace/install/setup.bash' >> /startup.sh && \
    echo 'echo "Starting AI-Powered Swarm Quadcopter System..."' >> /startup.sh && \
    echo 'ros2 launch full_system_integration complete_system.launch.py' >> /startup.sh && \
    chmod +x /startup.sh

# Expose ports
EXPOSE 8080 11311

# Set the default command
CMD ["/startup.sh"] 