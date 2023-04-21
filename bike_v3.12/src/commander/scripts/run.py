#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Float64
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose, Twist
import math
from simple_pid import PID

pole_twist = Twist()
y_angle = 0
y_angular=0

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    ans = roll_x
    return ans # in radians

def get_cart_pose(data):
    global pole_twist, y_angle, y_angular
    ind_pitch = data.name.index('motor_0419::base_link')
    state = data.pose[ind_pitch]
    y_angle = euler_from_quaternion(state.orientation.x, state.orientation.y, state.orientation.z, state.orientation.w)
    state = data.twist[ind_pitch]
    y_angular = state.angular.x

def commander():
    global pub_back, y_angular, y_angle
    pid=PID()
    Kp_p = 8.06
    # Ki_p = 6.18
    Kd_p = 0.038
    pid.tunings = (Kp_p, 0, Kd_p)
    time_interval = 0.001
    pub_back = rospy.Publisher('/back_fork_position_controller/command', Float64, queue_size = 10)
    y_angle = 0
    y_angular=0
    while not rospy.is_shutdown():
        time1 = time.time()

        # error_pos = target_yaw_angle - y_angle
        # integral_yaw_error += (error_pos + last_error_pos)*time_interval/2
        # yaw_displacement_sum += abs(y_angle)
        # effort_pos = -(Kp_p*error_pos  + 
        #                Ki_p*integral_yaw_error +
        #                Kd_p*(error_pos - last_error_pos)/time_interval)   

        # effort = effort_pos    

        # last_error_pos = error_pos
        # effort_pos = (Kp_p*y_angle + Kd_p*y_angular)
        # pub_back.publish(effort_pos)
        pub_back.publish(pid(y_angle))
        time2 = time.time()
        interval = time2 - time1
        if(interval < time_interval):
            time.sleep(time_interval - interval)

if __name__ == '__main__':
    rospy.init_node('commander', anonymous=True)
    rospy.Subscriber("/gazebo/link_states", LinkStates, get_cart_pose)
    commander()
    rospy.spin()
