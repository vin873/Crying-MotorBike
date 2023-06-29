#!/usr/bin/env python3

import rospy
import time
from random import randint
from std_msgs.msg import Float64, Int32
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import Imu
import math
from simple_pid import PID

pole_twist = Twist()
y_angle = 0
y_angular=0
y_angle_imu=0 
y_angular_imu=0
startFlag=0
fresh=1
period=10

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    ans = roll_x
    return ans # in radians

def imucallback(msg):
    global y_angle, y_angular, y_angle_imu, y_angular_imu
    y_angle_imu = euler_from_quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
    y_angular_imu=msg.angular_velocity.x
    # rospy.loginfo("%f", data)

def get_cart_pose(data):
    global pole_twist, y_angle, y_angular
    ind_pitch = data.name.index('motor_0419::base_link')
    state = data.pose[ind_pitch]
    y_angle = euler_from_quaternion(state.orientation.x, state.orientation.y, state.orientation.z, state.orientation.w)
    state = data.twist[ind_pitch]
    y_angular = state.angular.x

def startCallback(msg):
    global startFlag
    startFlag=msg.data

def freshCallback(msg):
    global fresh
    fresh= msg.data    

def periodCallback(msg):
    global period
    period= msg.data  

def commander():
    global pub_back, y_angular, y_angle, y_angle_imu, y_angular_imu, startFlag, fresh
    pid=PID()
    Kp_p = 8.1282438
    # Ki_p = 6.18
    Kd_p = 0.0347
    pid.tunings = (Kp_p, 0, Kd_p)
    time_interval = 0.001
    pub_back = rospy.Publisher('/back_fork_position_controller/command', Float64, queue_size = 10)
    pub_prbs = rospy.Publisher('prbs_data', Float64, queue_size = 10)
    y_angle = 0
    y_angular=0
    prbs_data=0
    lastD=0
    y_reference=0
    prbs_time = time.time()
    while not rospy.is_shutdown():
        time1 = time.time()
        if(time.time()-prbs_time>=fresh):
            prbs_data=(randint(0,1)-0.5)*2*0.01*startFlag
            y_reference=prbs_data
            # prbs_data = (math.sin(time.time()/period))*2*0.01*startFlag
            prbs_time = time.time()
        pub_prbs.publish(prbs_data)
        # print("ang:",y_angle,"imu_ang:",y_angle_imu,"   acc:",y_angular,"acc_imu:",y_angular_imu)
        # pub_back.publish(-(Kp_p*y_angle + Kd_p*y_angular)+prbs_data)
        pub_back.publish(-(Kp_p*(y_angle-y_reference) - Kp_p*((y_angle-y_reference)-lastD)))
        lastD=y_angle-y_reference
        time2 = time.time()
        interval = time2 - time1
        if(interval < time_interval):
            time.sleep(time_interval - interval)

if __name__ == '__main__':
    rospy.init_node('commander', anonymous=True)
    rospy.Subscriber("/gazebo/link_states", LinkStates, get_cart_pose)
    rospy.Subscriber("/imu", Imu, imucallback)
    rospy.Subscriber("startornot", Int32, startCallback)
    rospy.Subscriber("refreshRate", Float64, freshCallback)
    rospy.Subscriber("period", Float64, periodCallback)
    commander()
    rospy.spin()
