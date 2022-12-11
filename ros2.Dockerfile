FROM osrf/ros:humble-desktop

ENV ROS2_WS /home/ros2_ws

RUN mkdir -p ${ROS2_WS}

WORKDIR ${ROS2_WS}

RUN apt update
RUN apt install ros-humble-navigation2 -y
RUN apt install ros-humble-nav2-bringup -y

# Source the setup.bash file to allow you tu use ros2 commands
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc


CMD ["bash"]