#! /usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray

pry = [0.0, 0.0, 0.0]

def pry_callback(msg):
    for i in range(3):
        pry[i] = msg.data[i]
    print("inclination :", format(msg.data[0], '.5f'), "  p :", format(msg.data[1], '.5f'), "  r :", format(msg.data[2], '.5f'), "  y :", format(msg.data[3], '.5f'))

def listener():
    rospy.init_node("imu_data")
    rospy.Subscriber("imu_pry", Float64MultiArray, pry_callback)
    rospy.spin()

if __name__=="__main__":
    try:
        listener()

    except rospy.ROSInterruptException:
        pass