# Use ROS 2 Humble as Base
FROM ad8857/ros-llm:llama3.1-8b
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
RUN apt-get update && apt-get install -y \
    ros-humble-rclpy \
    ros-humble-std-msgs \
    python3-colcon-common-extensions \
  && rm -rf /var/lib/apt/lists/*

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

# Copy Entry Point Scripts
COPY entrypoint_scripts/ /entrypoint_scripts
RUN chmod +x /entrypoint_scripts/*.sh