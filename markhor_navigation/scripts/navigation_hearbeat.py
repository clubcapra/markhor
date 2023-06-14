#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseActionGoal
from std_msgs.msg import String

class HeartbeatPublisher:
    """
    Publishes a heartbeat message to the /heartbeat topic whenever a navigation goal is received.
    This will allow the robot to keep moving even if the web ui isn't connected.
    """
    def __init__(self):
        rospy.init_node('heartbeat_publisher', anonymous=True)
        self.heartbeat_pub = rospy.Publisher('heartbeat', String, queue_size=10)
        rospy.Subscriber('move_base/goal', MoveBaseActionGoal, self.callback)
        self.rate = rospy.Rate(1) # 1 Hz

    def callback(self, data):
        rospy.loginfo("Navigation goal received. Sending heartbeat.")
        self.heartbeat_pub.publish("Heartbeat")

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        heartbeat_publisher = HeartbeatPublisher()
        heartbeat_publisher.run()
    except rospy.ROSInterruptException:
        pass
