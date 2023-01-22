FROM osrf/ros:humble-desktop


# update
RUN apt-get update -y


# install cyclonedds
RUN apt install ros-humble-rmw-cyclonedds-cpp -y -qq

ENV RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
ENV HOST_ADDR="localhost"

COPY ./docker/dds.xml /root/dds.xml
ENV CYCLONEDDS_URI=/root/dds.xml



CMD ["ros2", "topic", "echo", "/joy"]