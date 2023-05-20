# markhor_slam

The package markhor_slam uses rtabmap_ros to perform the mapping. The launch file available in this package allow the Markhor to create multiple kind of map by itself. There is two possibility to start the slam on markhor. You can either start it in the simulation (Gazebo) or on the real robot.


## Starting the slam on gazebo

The easiest way to start the slam on gazebo is to use the following command :

```bash
roslaunch markhor_slam markhor_slam.launch
```

This command will convert the pointcloud to laserscan so that markhor will be able to produce a 2D map from the lidar.

## Starting the slam on the real robot

To start the slam on the real robot, you need to also start the sensors (IMU and velodyne). To ease the development process, you can use the markhor_slam wrapper the also start both sensor and active sensor fusion.


```bash
roslaunch markhor_slam markhor_slam_real.launch
```

Doing this allow the slam to support the 3D camera (ZED2).