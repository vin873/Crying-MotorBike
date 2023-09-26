#! /usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray

def pid_callback(msg):
    print("pid  :", format(msg.data[0], '+.5f'))
    print("motorError  :", format(msg.data[1], '+.5f'))
    print("motorErrorI  :", format(msg.data[2], '+.5f'))
    print("motorErrorD  :", format(msg.data[3], '+.5f'))
    print("\n\n")

def listener():
    rospy.init_node("listener")
    rospy.Subscriber("pid_pub", Float64MultiArray, pid_callback)
    rospy.spin()

if __name__=="__main__":
    try:
        listener()

    except rospy.ROSInterruptException:
        pass