# Use ROS 2 Humble as Base
FROM osrf/ros:humble-desktop

# Use Bash instead of Dash for better compatibility
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# Clean up any broken ROS sources and prepare for clean install
RUN rm -f /etc/apt/sources.list.d/ros2.list \
    && rm -f /etc/apt/sources.list.d/ros2.sources \
    && rm -f /etc/apt/trusted.gpg.d/ros-archive-keyring.gpg

# Install tooling and setup the new GPG key and repo
RUN apt-get update && apt-get install -y curl gnupg lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update

# Install ROS 2 packages
RUN apt-get install -y \
    ament-cmake \
    python3-pip \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rqt-image-view \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-rclpy \
    ros-humble-turtlesim \
    ros-humble-tf2-tools \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-xacro && \
    rm -rf /var/lib/apt/lists/*

# Install AI/Chat Model Dependencies
RUN pip install --no-cache-dir \
    numpy \
    scipy \
    pandas \
    openai \
    jpl-rosa \
    langchain_openai \
    langchain_community \
    torch \
    transformers \
    flask \
    fastapi \
    uvicorn

# Install ROS packages and tools
RUN apt-get update && apt-get install -y \
    ros-humble-tf2-tools \
    ros-humble-control-msgs 

# Install Python packages
RUN pip install --no-cache-dir \
    transforms3d \
    teleop

# Optional: Install PyTorch with CUDA 11.8 support (or switch to CPU-only)
RUN pip install --no-cache-dir \
    torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# Set up ROS 2 Workspaces
# Build colcon_ws
# COPY colcon_ws/src/ /colcon_ws/src/
# WORKDIR /colcon_ws
# RUN apt-get update && rosdep update && \
#     rosdep install --from-paths /colcon_ws/src --ignore-src -r -y
# RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install

# Build overlay_ws
COPY ./overlay_ws/src/ /overlay_ws/src/
WORKDIR /overlay_ws
RUN apt-get update && rosdep update && \
    rosdep install --from-paths /overlay_ws/src --ignore-src -r -y
# RUN source /colcon_ws/install/setup.bash && colcon build --symlink-install
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install


# Source in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
# RUN echo "source /colcon_ws_ws/install/setup.bash" >> /root/.bashrc
RUN echo "source /overlay_ws/install/setup.bash" >> /root/.bashrc

# Copy CycloneDDS Config for ROS 2 Middleware
COPY cyclonedds/config.xml /config.xml

# Copy Entry Point Scripts
COPY entrypoint_scripts/ /entrypoint_scripts
RUN chmod +x /entrypoint_scripts/*.sh
