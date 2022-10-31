FROM ros:melodic

ENV ROS_ROOT /root/markhor_ws

# Create a catkin workspace
RUN mkdir -p /root/markhor_ws/src
WORKDIR /root/markhor_ws/

RUN cd src

# Copy the repository into the container
COPY . .

# Update the package list
RUN apt-get update

# Clone capra_estop
RUN git clone https://github.com/clubcapra/capra_estop.git

# Install Gazebo
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN apt install wget -y
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN apt-get update
RUN apt-get install gazebo9 -y
RUN apt install libgazebo9-dev -y

# Install RQT
RUN apt-get install ros-melodic-rqt -y

# Install dependencies
RUN cd .. && rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd  /root/markhor_ws; catkin_make'

COPY scripts/ros_entrypoint.sh /ros_entrypoint.sh

# Source the setup file
RUN sed -i \
    's/ros_env_setup="\/opt\/ros\/$ROS_DISTRO\/setup.bash"/ros_env_setup="${ROS_ROOT}\/devel\/setup.bash"/g' \
    /ros_entrypoint.sh && \
    cat /ros_entrypoint.sh


RUN echo "source /root/markhor_ws/devel/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]