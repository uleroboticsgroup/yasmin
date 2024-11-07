# use the official ROS 2 humble base image
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO} AS deps

# set the working directory and copy files
WORKDIR /root/ros2_ws
SHELL ["/bin/bash", "-c"]
COPY . /root/ros2_ws/src

# install dependencies
RUN source /opt/ros/${ROS_DISTRO}/setup.bash
RUN apt-get update
RUN rosdep update && rosdep install --from-paths src --ignore-src -r -y
RUN if [ "$ROS_DISTRO" = "foxy" ] || [ "$ROS_DISTRO" = "galactic" ] || [ "$ROS_DISTRO" = "rolling" ]; then \
    apt install -y ros-$ROS_DISTRO-example-interfaces \
                   ros-$ROS_DISTRO-action-tutorials-interfaces; \
    fi

# colcon the ws
FROM deps AS builder
ARG CMAKE_BUILD_TYPE=Release
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build

# source the ROS 2 setup file
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# run a default command, e.g., starting a bash shell
CMD ["bash"]
