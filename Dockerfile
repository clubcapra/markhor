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
RUN catkin_make