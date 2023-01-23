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
To use the gazebo simulation you need to update the gazebo version on your machine to atleast `9.14`, the default version included with melodic should be fine.

## In the tutorial instead of downloading `gazebo11` download `gazebo9`.

For this you need to follow the `step-by-step` alternative installation method over at : http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install


## Dependencies installation
### Rosdep managed dependencies
Some of the dependencies require manual installation, first, install the automatic dependencies using rosdep (running from the workspace root): 
`rosdep install --from-paths src --ignore-src -r -y `
### Gazebo_ros_tracked_vehicle_interface

To allow the tracks to be controlled with ros, you will need to build the gazebo_ros_tracked_vehicle_interface plugin, it can be found here : 
https://github.com/lprobsth/gazebo_ros_tracked_vehicle_interface

Clone it in your catkin workspace src folder, then you will need to update the gazebo sources following those steps: 

Add the package list:
```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
```

Add the lists key:
```bash
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

Rebuild the local index:
```bash
sudo apt-get update
```

Upgrade packages - gazebo9 should be included:
```bash
sudo apt-get upgrade
```

**MAKE SURE YOU DELETE YOUR DEVEL AND BUILD FOLDERS AFTER INSTALLING THE LIBS**
Then use catkin make and make sure there are no errors.
### OPTIONAL : AWS Small house world

If you want to use the small house world, you need to clone it in your catkin workspace and do a catkin make.

https://github.com/aws-robotics/aws-robomaker-small-house-world

### Vmware 3d acceleration
If using vmware, gazebo won't launch when using 3d acceleration, 3d acceleration is required to get a decent framerate in gazebo to fix it, run the folowing command in the linux VM
```bash
echo "export SVGA_VGPU10=0" >> ~/.profile
```


# Running the simulation

### Start the simulation with tracks
`roslaunch markhor_gazebo test_world.launch`


**Note**: Make sure you have the repository built & sourced.

## Running the simulation with Markhor
### Start sending cmd_vel command through a joystick 
`roslaunch markhor_bringup teleop_twist_joy.launch`

### Start sending cmd_vel command through the keyboard 
`rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/markhor/diff_drive_controller/cmd_vel`

## Controlling the flippers axe through ROS
When running the simulation with the tracks you can control the flippers axe with this command:

`rostopic pub -1 /markhor/flipper_<fl|fr|rl|rr>_position_controller/command std_msgs/Float64 "data: <0 = 180deg -3 = -180deg>"`

In the future this control will be done with a hardware interface.
