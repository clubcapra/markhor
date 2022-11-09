Depends on aws smal house world https://github.com/aws-robotics/aws-robomaker-small-house-world
Depends on velodyne simulator https://github.com/lmark1/velodyne_simulator

pointcloud_to_laserscan

hector_slam

if using vmware, gazebo won't launch when using 3d acceleration, to fix it, run the folowing command 
```bash
echo "export SVGA_VGPU10=0" >> ~/.profile
```

To use the gazebo simulation, you need the ros tracked vehicule interface gazebo plugin found here: 
https://github.com/lprobsth/gazebo_ros_tracked_vehicle_interface
This allows the track simulation plugin to be controlled with ros

Pull it in your workspace and build it using catkin.


You need to install gazebo libraries to build the plugin, here are the instructions from the plugin readme to do so :

**1. (optional) Setup the OSRF package list and upgrade gazebo9**

Only if you have not yet added the package list to the the apt sources. Have a look at http://gazebosim.org/install which also describes the installation of gazebo binary packages. (page is offline at the time of writing - so commands are listed below)

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


