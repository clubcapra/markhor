#!/usr/bin/env python

import rospy
import actionlib
from markhor_navigation.msg import ExplorationAction, ExplorationFeedback, ExplorationResult
from explore_lite.msg import ExploreTaskAction, ExploreTaskGoal
from rospy.timer import TimerEvent

class ExplorationActionServer(object):

    def __init__(self):
        self.server = actionlib.SimpleActionServer('exploration', ExplorationAction, self.execute, False)
        self.exploration_client = actionlib.SimpleActionClient('explore_server', ExploreTaskAction)
        self.exploration_client.wait_for_server()
        self.server.start()

    def execute(self, goal):
        r = rospy.Rate(1)
        success = True

        # start a timer for exploration
        self.timer = rospy.Timer(goal.exploration_time, self.timer_callback, oneshot=True)

        # start exploration
        exploration_goal = ExploreTaskGoal()  # adjust this as necessary
        self.exploration_client.send_goal(exploration_goal)

        while not rospy.is_shutdown() and self.server.is_active():
            if self.server.is_preempt_requested():
                rospy.loginfo('Exploration was preempted')
                self.server.set_preempted()
                self.exploration_client.cancel_all_goals()
                success = False
                break

            # send feedback
            feedback = ExplorationFeedback()
            feedback.feedback = "Exploring..."
            self.server.publish_feedback(feedback)

            r.sleep()

        # after the exploration
        if success:
            rospy.loginfo('Exploration completed successfully')
            result = ExplorationResult()
            result.success = success
            self.server.set_succeeded(result)

    def timer_callback(self, event):
        rospy.loginfo("Exploration time is over")
        self.server.preempt_request = True


if __name__ == '__main__':
    rospy.init_node('exploration')
    server = ExplorationActionServer()
    rospy.spin()
