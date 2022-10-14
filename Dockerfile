FROM ros:melodic

# Create a catkin workspace
RUN mkdir -p /root/markhor_ws/src
WORKDIR /root/markhor_ws/src

# Copy the repository into the container
COPY . .

# Update the package list
RUN apt-get update

# Clone capra_estop
RUN git clone https://github.com/clubcapra/capra_estop.git

# Install dependencies
RUN cd .. && rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd  /root/markhor_ws; catkin_make'

# Source the setup file
RUN echo "source /root/markhor_ws/devel/setup.bash" >> ~/.bashrc