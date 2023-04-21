#!/usr/bin/env python3
import rospy, time
from deap import base, creator, tools
import random
import datetime
from std_msgs.msg import Float64
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose, Twist
from std_srvs.srv import Empty
import math
from simple_pid import PID
state = Pose()
y_angle = 0

pub_ang = rospy.Publisher('/bikeAng', Float64, queue_size = 10)

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    return(roll_x)

def get_pose(data):
    global y_angle, state
    ind_pitch = data.name.index('motor_0419::base_link')
    state = data.pose[ind_pitch]
    y_angle = euler_from_quaternion(state.orientation.x, state.orientation.y, state.orientation.z, state.orientation.w)
    pub_ang.publish(y_angle)

def listener():
    rospy.init_node('pubAng', anonymous=True)
    rospy.Subscriber("/gazebo/link_states", LinkStates, get_pose)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()

    except rospy.ROSInterruptException:
        pass