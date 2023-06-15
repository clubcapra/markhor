# Script that starts gazebo with the correct world file

docker run -it --rm --gpus all --privileged -p 9090:9090 -v markhor_gazebo:/root/markhor_ws/src/markhor_gazebo/ -e DISPLAY=host.docker.internal:0.0 -e GAZEBO_GUI=true -v /tmp/.X11-unix:/tmp/.X11-unix ghcr.io/clubcapra/markhor/markhor:latest bash -c "roslaunch markhor_gazebo test_world.launch"
