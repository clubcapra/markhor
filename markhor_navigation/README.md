# Markhor Navigation

## Default navigation

The markhor navigation package is used to perform the navigation of the robot. It uses the ROS navigation stack to perform the navigation. The launch file available in this package (markhor_nav.launch) allow  Markhor to use move base and some planner with preconfigured parameters. The navigation can work in simulation and in real life. In both case, the navigation will use the map created by the slam to perform the navigation, thus the slam need to be running before starting the navigation.

To use the navigation a user can open RVIZ and select point to go on the map.


## Frontier exploration

Markhor navigation also possess a frontier exploration node. This node allow the robot to explore the environment by itself. When you start the navigation, you also start the frontier exploration (and return) node. This node will make the robot discover the environment for a given time. After this time, the robot will return to its initial position (From where the service was called).

To call the frontier exploration, you can use the following command :

```bash
rosservice call /start_exploration "timeout: 80"
```

In this case, the robot will explore the environment for 80 seconds. Note that the navigation is slow and 80 seconds is a very short time for the robot to explore the environment. However, the more time you give to the robot, the robot can get stuck and stop the exploration. This will not cancel the exploration and you'll lose some time. 

To stop the service, you can use the following command : 


```bash
rosservice call /stop_exploration
```