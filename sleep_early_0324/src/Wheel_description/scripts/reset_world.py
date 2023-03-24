#! /usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int64
from std_srvs.srv import Empty
import math

def talker():
    rospy.init_node('reset_world', anonymous=True)
    pub = rospy.Publisher('/resetting', Int64, queue_size=100)
    pub.publish(1)
    rospy.wait_for_service('/gazebo/reset_simulation')
    reset_world = rospy.ServiceProxy('/gazebo/reset_simulation',Empty)
    reset_world()
    rospy.sleep(0.3)
    pub.publish(0)

if __name__ == '__main__' :
    try:
        talker()
    except rospy.ROSInterruptException:
        pass