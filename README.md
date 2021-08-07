# markhor

## Installing dependancies
You can use this command inside the `markhor_ws` to install the project dependancies. This might not take into account the Gazebo upgrade that we need to do to run the simulation correctly.
`rosdep install --from-paths src --rosdistro melodic -y`


# markhor gazebo
To use the gazebo simulation you need to update the gazebo version on your machine to atleast `9.14`.

## In the tutorial instead of download `gazebo11` download `gazebo9`.

For this you need to follow the `step-by-step` alternative installation method over at : http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install



Also make sure you have the dependancies download. You can make sure by using the command : `rosdep install --from-paths src --ignore-src -r -y `


# Running the simulation 
The simulation suffers from a missing implementation of the link between the ros_control and the SimpleTrackVehiculePlugin. Although it is fit to simulate and control the robot with the arrow keys it is not possible to control it through ROS.

This necessitates a workaround when testing the robot with the UI or with a joystick. We repurpose the wheel from the Clearpath Husky robot in the simulation.

**Note**: Make sure you have the repository build & source.

## Running the simulation with Markhor with husky wheels (joystick/keyboard with ROS)
### Start the simulation with husky wheels
`roslaunch markhor_gazebo test_world_husky.launch`

### Start sending cmd_vel command through a joystick 
`roslaunch markhor_bringup teleop.launch`

### Start sending cmd_vel command through the keyboard 
`rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/markhor/diff_drive_controller/cmd_vel`

## Running the simulation with Markhor with tracks (keyboard without ROS)
### Start the simulation with tracks
`roslaunch markhor_gazebo test_world.launch`

### How to move the robot
To move the robot, first select Markhor inside the simulation by clicking on it and use your arrow keys to control it.

## Controlling the flippers axe through ROS
When running the simulation with the tracks you can control the flippers axe with this command:

`rostopic pub -1 /markhor/flipper_<fl|fr|rl|rr>_position_controller/command std_msgs/Float64 "data: <0 = 180deg -3 = -180deg>"`

In the future this control will be done with a hardware interface.
