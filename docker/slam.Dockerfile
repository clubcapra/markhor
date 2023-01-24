FROM osrf/ros:humble-desktop


# update
RUN apt-get update -y


# install cyclonedds
RUN apt install ros-humble-rmw-cyclonedds-cpp -y -qq

# install slam
RUN apt install -y ros-humble-slam-toolbox


ENV RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
ENV HOST_ADDR="localhost"

COPY ./dds.xml /root/dds.xml
ENV CYCLONEDDS_URI=/root/dds.xml



CMD ["ros2", "launch", "slam_toolbox", "online_async_launch.py", "params_file:=/mapper_params_online_async.yaml"]