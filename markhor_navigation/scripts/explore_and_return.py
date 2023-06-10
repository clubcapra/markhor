#!/usr/bin/env python

import rospy
import subprocess
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from nav_msgs.msg import Odometry
from markhor_navigation.srv import StartExploration, StartExplorationResponse
from geometry_msgs.msg import Pose


def handle_start_exploration(req):
    """
    Service server that starts the explore_lite node and returns the robot to its original position after a specified amount of time.
    The original position is the position of the robot when the service is called.
    """
    # Get the timeout from the request
    timeout = req.timeout

    # Get the current position to use as the return position
    rospy.loginfo("Getting current position to use as return position...")
    current_pose_msg = rospy.wait_for_message('/markhor/odometry/filtered', Odometry)
    return_position = current_pose_msg.pose.pose

    # Create action client for move_base
    rospy.loginfo("Connecting to move_base server...")
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    rospy.loginfo("Connected to move_base server")

    # Start explore_lite node
    rospy.loginfo("Starting exploration...")
    explore_process = subprocess.Popen(["roslaunch", "explore_lite", "explore_costmap.launch"])

    # Wait for the specified amount of time
    rospy.loginfo("Exploring for {} seconds...".format(timeout))
    time.sleep(timeout)

    # Stop explore_lite node
    rospy.loginfo("Stopping exploration...")
    explore_process.terminate()

    # Send goal to move_base to move back to the return position
    rospy.loginfo("Returning to return position...")
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = return_position
    client.send_goal(goal)

    # Wait for the action to complete
    client.wait_for_result()

    return StartExplorationResponse(True, "Exploration started and return position set successfully")

if  __name__ == '__main__':
    try:
        # Initialize the ros node
        rospy.init_node('explore_and_return', anonymous=True)

        # Start the service server
        rospy.Service('start_exploration', StartExploration, handle_start_exploration)

        # Keep the node running until it's shut down
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
