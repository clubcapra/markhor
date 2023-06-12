#! /bin/bash

# Script that starts gazebo with the correct world file
docker run -it --gpus all --privileged --net host -e DISPLAY=:1 -v /tmp/.X11-unix:/tmp/.X11-unix ghcr.io/clubcapra/markhor/markhor:latest bash -c "Xvfb :1 -screen 0 1024x768x16 & sleep 1; roslaunch markhor_gazebo test_world.launch"