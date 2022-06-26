# markhor

## Installing dependencies
You can use this command inside the `markhor_ws` to install the project dependencies. This might not take into account the Gazebo upgrade that we need to do to run the simulation correctly.
`rosdep install --from-paths src --rosdistro melodic -y`

The ros_package `capra_estop` is essential to launch markhor. It should be located in the `markhor_ws/src` directory.
If the module is not there, markhor will not launch and you will see the following error message : `Resource not found: capra_estop`

### Package dependencies
- [usb_cam]
- [capra_estop](https://github.com/clubcapra/capra_estop)

### Other intented use packages
Altough not required, these packages are also expected to be installed alongside markhor. If not present, error messages will appear when launching the robot.
- [capra_audio_common](https://github.com/clubcapra/capra_audio_common)
- [TPV_controller](https://github.com/clubcapra/TPV_controller)
- [capra_vision_visp](https://github.com/clubcapra/capra_vision_visp)
- [capra_thermal_cam](https://github.com/clubcapra/capra_thermal_cam)
- [ovis](https://github.com/clubcapra/ovis)
- [ovis_robotiq_gripper](https://github.com/clubcapra/ovis_robotiq_gripper)
<!-- Launch files not updated for these packages
- [capra_hazmat_detection](https://github.com/clubcapra/capra_hazmat_detection) currently named yolo_hazmat
-->

# Launching the robot

There are multiple launch configurations depending on your needs. The intended use being that you launch them through [capra_web_ui's](https://github.com/clubcapra/capra_web_ui) functionality that require the [capra_launch_handler](https://github.com/clubcapra/capra_launch_handler) installed on the platform. For reference, the terminal commands will also be added to each described configuration.

**Note**: Make sure you have the repository built & sourced.

- `markhor_parkour.launch` : This launch file launches every movement and camera related nodes.
- `markhor_observation.launch` : This launch file aims to launch every sensor equipped and the robot arm with its gripper, ovis in this case.
- `markhor_base.launch` : While the two other files are made to be an optimised for competition setup, the `base` version launches
everything simultaneously for demonstration purposes.  

# markhor gazebo
To use the gazebo simulation you need to update the gazebo version on your machine to atleast `9.14`.

## In the tutorial instead of downloading `gazebo11` download `gazebo9`.

For this you need to follow the `step-by-step` alternative installation method over at : http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install


Also make sure you have the dependencies downloaded. You can make sure by using the command : `rosdep install --from-paths src --ignore-src -r -y `


# Running the simulation
The simulation suffers from a missing implementation of the link between the ros_control and the SimpleTrackVehiculePlugin. Although it is fit to simulate and control the robot with the arrow keys it is not possible to control it through ROS.

This necessitates a workaround when testing the robot with the UI or with a joystick. We repurpose the wheel from the Clearpath Husky robot in the simulation.

**Note**: Make sure you have the repository built & sourced.

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
