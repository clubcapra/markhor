#!/usr/bin/env python

import rospy
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

def takeover() :
    print()
    
def record_connection() :
    print()
    

tookover = False
    

if __name__ == '__main__':
    try:
        # Initialize the ros node
        rospy.init_node('nav_takeover', anonymous=True)

        # Set the rate of the node
        rate = rospy.Rate(10)  # 10 Hz
        
        start_nav_srv = rospy.Service('start_nav_takeover', Trigger, takeover)
        record_points_sub = rospy.Subscriber('/wifi_signal_points', PointCloud2, record_connection, queue_size=1)

        # Keep the node running until it's shut down
        while not rospy.is_shutdown():

            # Sleep for the remainder of the loop to enforce the rate
            rate.sleep()
    except rospy.ROSInterruptException:
        pass