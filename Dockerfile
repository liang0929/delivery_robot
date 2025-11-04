# Use the ROS Humble base image
FROM ros:humble-ros-base

# Set the shell to bash
SHELL ["/bin/bash", "-c"]

# Install necessary tools
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-vcs-importer \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install dependencies from package.xml files
RUN apt-get update && apt-get install -y \
    ros-humble-ament-cmake-python \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    ros-humble-std-srvs \
    ros-humble-rclcpp \
    && rm -rf /var/lib/apt/lists/*

# Create a workspace
WORKDIR /ros2_ws

# Copy the source code
COPY src ./src

# Install dependencies using rosdep
RUN rosdep init && rosdep update
RUN rosdep install -i --from-path src --rosdistro humble -y

# Build the workspace
RUN . /opt/ros/humble/setup.bash && colcon build

# Set up the entrypoint
COPY <<EOF /ros_entrypoint.sh
#!/bin/bash
set -e
source "/opt/ros/humble/setup.bash"
source "/ros2_ws/install/setup.bash"
exec "$@"
EOF
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
