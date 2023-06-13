#!/usr/bin/env python

import rospy
import subprocess
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from nav_msgs.msg import Odometry
from markhor_navigation.srv import StartExploration, StartExplorationResponse
from geometry_msgs.msg import Pose
import rosnode

def distance_between_positions(position1, position2):
    """
    Calculates the euclidean distance between two positions (2 dimensions).
    """
    dx = position1.x - position2.x
    dy = position1.y - position2.y
    return (dx**2 + dy**2)**0.5

def handle_start_exploration(req):
    """
    Service server that starts the explore_lite node and returns the robot to its original position after a specified amount of time.
    The original position is the position of the robot when the service is called.
    """
    # Get the timeout from the request
    timeout = req.timeout

    # Is the exploration running
    is_running = True

    # Get the current position to use as the return position
    rospy.loginfo("Getting current position to use as return position...")
    current_pose_msg = rospy.wait_for_message('/markhor/odometry/filtered', Odometry)
    return_pose = current_pose_msg.pose.pose
    last_position = current_pose_msg.pose.pose.position  # Keep track of the last position

    # Create action client for move_base
    rospy.loginfo("Connecting to move_base server...")
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    rospy.loginfo("Connected to move_base server")

    # Start explore_lite node
    rospy.loginfo("Starting exploration...")
    explore_process = subprocess.Popen(["roslaunch", "explore_lite", "explore_costmap.launch"])
    start_time = rospy.Time.now()

    max_intialisation_wait_time = 10
    is_initialised = False
    # Wait for the explore_lite node to start (or wait initialisation time)
    rospy.loginfo("Waiting for explore_lite node to start...")
    while not is_initialised and (rospy.Time.now() - start_time).to_sec() < max_intialisation_wait_time:
        if '/explore' in rosnode.get_node_names():
            is_initialised = True
        time.sleep(0.5)  # check every half second



    # Wait for the specified amount of time
    rospy.loginfo("Exploring for {} seconds...".format(timeout))

    while is_running and is_initialised:
        
        time.sleep(5)  # check the robot's position every 5 seconds

        # Get the robot's current position
        current_pose_msg = rospy.wait_for_message('/markhor/odometry/filtered', Odometry)
        current_position = current_pose_msg.pose.pose.position

        # If the robot hasn't moved for a certain amount of time, end the exploration
        if distance_between_positions(last_position, current_position) < 0.05:  # If the robot didn't move more than 5 cm in 5 seconds
            rospy.logerr("Robot is stuck, stopping exploration...")
            is_running = False

        if (rospy.Time.now() - start_time).to_sec() > timeout:
            rospy.loginfo("Exploration time finished...")
            is_running = False

        last_position = current_position  # Update the last position

    # Stop explore_lite node
    rospy.loginfo("Stopping exploration...")
    explore_process.terminate()

    # Send goal to move_base to move back to the return position
    rospy.loginfo("Returning to return position...")
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'odom'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = return_pose
    client.send_goal(goal)

    # Wait for the action to complete
    client.wait_for_result()

    return StartExplorationResponse(True, "Exploration started and return position set successfully")


if __name__ == '__main__':
    try:
        # Initialize the ros node
        rospy.init_node('explore_and_return', anonymous=True)

        # Start the service server
        rospy.Service('start_exploration', StartExploration, handle_start_exploration)

        # Set the rate of the node
        rate = rospy.Rate(5)  # 10 Hz

        # Keep the node running until it's shut down
        while not rospy.is_shutdown():
            # Do any processing you need to do here

            # Sleep for the remainder of the loop to enforce the rate
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
