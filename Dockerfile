# Use the official ROS 2 humble base image
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO} AS deps

# Set the working directory and copy files
WORKDIR /root/ros2_ws
SHELL ["/bin/bash", "-c"]
COPY . /root/ros2_ws/src

# Install dependencies
RUN source /opt/ros/${ROS_DISTRO}/setup.bash
RUN apt-get update
RUN rosdep update --include-eol-distros && rosdep install --from-paths src --ignore-src -r -y

# Build the ws (colcon)
FROM deps AS builder
ARG CMAKE_BUILD_TYPE=Release
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build

# Source the ROS 2 setup file
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# Run a default command, e.g., starting a bash shell
CMD ["bash"]
