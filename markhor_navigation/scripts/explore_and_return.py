#!/usr/bin/env python

import rospy
import subprocess
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from nav_msgs.msg import Odometry

def explore_and_return():
    # Initialize the ros node
    rospy.init_node('explore_and_return', anonymous=True)

    # Create action client for move_base
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    rospy.loginfo("Connected to move_base server")

    # Get initial position
    initial_pose_msg = rospy.wait_for_message('/markhor/odometry/filtered', Odometry)
    initial_pose = initial_pose_msg.pose.pose
    rospy.loginfo("Initial position: {}".format(initial_pose))

    # Start explore_lite node
    rospy.loginfo("Starting exploration...")
    explore_process = subprocess.Popen(["roslaunch", "explore_lite", "explore_costmap.launch"])

    # Wait for specified amount of time
    time.sleep(60)  # replace 60 with the amount of time you want to explore

    # Stop explore_lite node
    rospy.loginfo("Stopping exploration...")
    explore_process.terminate()

    # Send goal to move_base to move back to initial position
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'  # assuming you're using map frame
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = initial_pose

    rospy.loginfo("Returning to initial position...")
    client.send_goal(goal)

    # Wait for the action to complete
    client.wait_for_result()

if __name__ == '__main__':
    try:
        explore_and_return()
    except rospy.ROSInterruptException:
        pass
