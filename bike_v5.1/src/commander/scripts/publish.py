#! /usr/bin/env python3

import rospy
import time
from std_msgs.msg import Float64
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
import math

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    ans = roll_x
    return ans # in radians

def get_cart_pose(data):
    global pole_twist, y_angle, y_angular, state
    ind_pitch = data.name.index('new_wheel::base_link')
    state = data.pose[ind_pitch]
    y_angle = euler_from_quaternion(state.orientation.x, state.orientation.y, state.orientation.z, state.orientation.w)
    state = data.twist[ind_pitch]
    y_angular = state.angular.x
    pubyangle = rospy.Publisher('yangle', Float64, queue_size = 10)
    pubyangular = rospy.Publisher('yangular', Float64, queue_size = 10)
    pubyangle.publish(y_angle)
    pubyangular.publish(y_angular)

def publisher():
    rospy.init_node("publish")
    rospy.Subscriber("/gazebo/link_states", LinkStates, get_cart_pose)
    rospy.spin()

if __name__=="__main__":
    try:
        publisher()

    except rospy.ROSInterruptException:
        pass