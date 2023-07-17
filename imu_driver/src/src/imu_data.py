#! /usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray

def imu_callback(msg):
    print("inclination :", format(msg.data[0], '+.5f'))
    print("roll  :", format(msg.data[1], '+.5f'), "  pitch :", format(msg.data[2], '+.5f'), "  yaw   :", format(msg.data[3], '+.5f'))
    print("ang_X :", format(msg.data[4], '+.5f'), "  ang_Y :", format(msg.data[5], '+.5f'), "  ang_Z :", format(msg.data[6], '+.5f'))
    print("acc_X :", format(msg.data[7], '+.5f'), "  acc_Y :", format(msg.data[8], '+.5f'), "  acc_Z :", format(msg.data[9], '+.5f'))
    print("\n\n")

def listener():
    rospy.init_node("imu_data")
    rospy.Subscriber("imu_data", Float64MultiArray, imu_callback)
    rospy.spin()

if __name__=="__main__":
    try:
        listener()

    except rospy.ROSInterruptException:
        pass
