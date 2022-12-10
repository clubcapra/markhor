FROM ros:melodic

ENV ROS_ROOT /root/markhor_ws

# Create a catkin workspace
RUN mkdir -p /root/markhor_ws/src
WORKDIR /root/markhor_ws/

# Update the package list
RUN apt-get update

# Install Gazebo
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN apt install wget -y
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN apt-get update
RUN apt-get install gazebo9 -y
RUN apt install libgazebo9-dev -y

# Install RQT
RUN apt-get install ros-melodic-rqt -y