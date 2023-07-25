#!/usr/bin/env python3
import rospy, time
from deap import base, creator, tools
import random
import datetime
from std_msgs.msg import Float64
from std_msgs.msg import String
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose, Twist
from std_srvs.srv import Empty
import math
from simple_pid import PID
state = Pose()
angle=0
fork=0
pstr = String()

pub_in = rospy.Publisher('/input', Float64, queue_size = 10)
pub_out = rospy.Publisher('/output', Float64, queue_size = 10)
pub_inout = rospy.Publisher('/inout', String, queue_size = 10)
pub_state = rospy.Publisher('/rollState', Float64, queue_size = 10)

# def euler_from_quaternion(x, y, z, w):
#     t0 = +2.0 * (w * x + y * z)
#     t1 = +1.0 - 2.0 * (x * x + y * y)
#     roll_x = math.atan2(t0, t1)
#     return roll_x # in radians

# def get_pose(data):
#     global y_angle, state,y_angular, pub_state
#     ind_pitch = data.name.index('motor_0419::base_link')
#     state = data.pose[ind_pitch]
#     y_angle = euler_from_quaternion(state.orientation.x, state.orientation.y, state.orientation.z, state.orientation.w)
#     state = data.twist[ind_pitch]
#     pubFloat=Float64()
#     pubFloat.data=state.angular.x
#     pub_state.publish(pubFloat)

def bicyCall(msg):
    global angle
    angle=msg.data

def forCall(msg):
    global fork
    fork=msg.data

def listener():
    global fork, angle
    rospy.init_node('pubTeamEpic', anonymous=True)
    rospy.Subscriber("/bikeAng", Float64, bicyCall)
    rospy.Subscriber("/back_fork_position_controller/command", Float64, forCall)
    while not rospy.is_shutdown():
        pstr = str(fork)+", "+str(angle)
        pub_inout.publish(pstr)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()

    except rospy.ROSInterruptException:
        pass