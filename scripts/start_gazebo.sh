#! /bin/bash

# Get the absolute path of the parent directory of the current script
PARENT_DIRECTORY="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Go to source directory
SRC_DIRECTORY="$(dirname "$(dirname "$PARENT_DIRECTORY")")"


# Script that starts gazebo with the correct world file
docker run -it --gpus all -v $SRC_DIRECTORY:/root/markhor_ws/src --privileged --net host -e DISPLAY=:1 -v /tmp/.X11-unix:/tmp/.X11-unix ghcr.io/clubcapra/markhor/markhor:latest bash -c "Xvfb :1 -screen 0 1024x768x16 & sleep 1; roslaunch markhor_gazebo small_house.launch"