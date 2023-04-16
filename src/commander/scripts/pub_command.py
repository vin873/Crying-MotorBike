#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Float64
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose, Twist
import math

pole_twist = Twist()
y_angle = 0

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    ans = roll_x
    return ans # in radians

def get_cart_pose(data):
    global pole_twist, y_angle
    ind_pitch = data.name.index('Wheel::back_link_1')
    state = data.pose[ind_pitch]
    y_angle = euler_from_quaternion(state.orientation.x, state.orientation.y, state.orientation.z, state.orientation.w)


def commander():
    global sentry_robot_vel, cart_pose, y_angular, cart_pose_x
    yaw_angle = 0
    target_yaw_angle = 0
    target_cart_pose_x = 0
    integral_position_error = 0
    integral_yaw_error = 0
    last_error_yaw = 0
    last_error_pos = 0
    yaw_displacement_sum = 0

    Kp_p = 0
    Ki_p = 6.18
    Kd_p = 5.5

    time_interval = 0.005
    pub_cart = rospy.Publisher('/Revolute_2_position_controller/command', Float64, queue_size = 10)

    while not rospy.is_shutdown():
        time1 = time.time()

        error_pos = target_yaw_angle - y_angle
        integral_yaw_error += (error_pos + last_error_pos)*time_interval/2
        yaw_displacement_sum += abs(y_angle)
        effort_pos = -(Kp_p*error_pos  + 
                       Ki_p*integral_yaw_error +
                       Kd_p*(error_pos - last_error_pos)/time_interval)   

        effort = effort_pos    

        last_error_pos = error_pos

        pub_cart.publish(effort)
        time2 = time.time()
        interval = time2 - time1
        if(interval < time_interval):
            time.sleep(time_interval - interval)

if __name__ == '__main__':
    rospy.init_node('commander', anonymous=True)
    rospy.Subscriber("/gazebo/link_states", LinkStates, get_cart_pose)
    commander()
    rospy.spin()
