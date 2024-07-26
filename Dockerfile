# Use the official ROS 2 humble base image
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO} as deps

# Set the working directory
WORKDIR /root/ros2_ws

SHELL ["/bin/bash", "-c"]

RUN source /opt/ros/${ROS_DISTRO}/setup.bash
RUN apt-get update \
    && apt-get -y --quiet --no-install-recommends install \
    gcc \
    git \
    python3 \
    python3-pip

COPY . /root/ros2_ws/src
RUN rosdep install --from-paths src --ignore-src -r -y

WORKDIR /root/ros2_ws/src

RUN pip3 install -r requirements.txt
# Run a default command, e.g., starting a bash shell
CMD ["bash"]

WORKDIR /root/ros2_ws
FROM deps as builder
ARG CMAKE_BUILD_TYPE=Release

RUN source /opt/ros/${ROS_DISTRO}/setup.bash
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build

# Source the ROS2 setup file
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# Run a default command, e.g., starting a bash shell
CMD ["bash"]
