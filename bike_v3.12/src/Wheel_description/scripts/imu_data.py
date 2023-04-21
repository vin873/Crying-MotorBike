#! /usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Int64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
import math
 
def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
   
    # t2 = +2.0 * (w * y - z * x)
    # t2 = +1.0 if t2 > +1.0 else t2
    # t2 = -1.0 if t2 < -1.0 else t2
    # pitch_y = math.asin(t2)
 
    # t3 = +2.0 * (w * z + x * y)
    # t4 = +1.0 - 2.0 * (y * y + z * z)
    # yaw_z = math.atan2(t3, t4)

    #ans = Vector3(roll_x, pitch_y, yaw_z)
    
    ans = roll_x

    return ans # in radians


data = 0.0000

def imucallback(msg):
    global data
    data = euler_from_quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
    # print(data)

def listener():
    rospy.init_node('imu_data', anonymous=True)
    rospy.Subscriber("/imu", Imu, imucallback)
    pub = rospy.Publisher('/Revolute_2_position_controller/command', Float64, queue_size=100)
    # pub2 = rospy.Publisher('/Revolute_5_position_controller/command', Float64, queue_size=100)
    rate = rospy.Rate(200)
    while not rospy.is_shutdown():
        pub.publish(-data)
        # pub2.publish(data)
        rate.sleep()

if __name__ == '__main__' :
    try:
        listener()
    except rospy.ROSInterruptException:
        pass