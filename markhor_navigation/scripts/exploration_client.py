#!/usr/bin/env python

import rospy
import actionlib
from markhor_navigation.msg import ExplorationAction, ExplorationGoal

rospy.init_node('exploration_client')
client = actionlib.SimpleActionClient('exploration', ExplorationAction)
client.wait_for_server()

goal = ExplorationGoal()
goal.exploration_time = rospy.Duration.from_sec(30)  # 30 minutes

client.send_goal(goal)
client.wait_for_result()

print("Exploration Result: ", client.get_result().success)