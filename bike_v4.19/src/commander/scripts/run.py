#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import Float64
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
import math
from simple_pid import PID

pole_twist = Twist()
y_angle = 0
y_angular=0
y_angle_imu=0 
y_angular_imu=0
y_reference=0
Kp_p = 14.7
Ki_p = 0
Kd_p = 1.197

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
    ind_pitch = data.name.index('new_wheel::base_link')
    state = data.pose[ind_pitch]
    y_angle = euler_from_quaternion(state.orientation.x, state.orientation.y, state.orientation.z, state.orientation.w)
    state = data.twist[ind_pitch]
    y_angular = state.angular.x

def refCallback(msg):
    global y_reference
    y_reference=msg.data

def KpCallback(msg):
    global Kp_p
    Kp_p=msg.data

def KiCallback(msg):
    global Ki_p
    Ki_p=msg.data

def KdCallback(msg):
    global Kd_p
    Kd_p=msg.data
    
def get_joint(data):
    global motorPos
    motorPos=data.position[1]

def commander():
    global pub_back, y_angular, y_angle, y_angle_imu, y_angular_imu, Kp_p, Ki_p, Kd_p
    time_interval = 0.001
    pub_back = rospy.Publisher('/back_fork_effort_controller/command', Float64, queue_size = 10)
    y_angle=0
    y_angular=0
    lastD=0
    yaw_int=0
    lastMotorError=0
    targetPos=0
    motorPos=0
    motorError=0
    motorErrorI=0
    motorErrorD=0

    while not rospy.is_shutdown():
        time1 = time.time()
        # print("ang:",y_angle,"imu_ang:",y_angle_imu,"   acc:",y_angular,"acc_imu:",y_angular_imu)
        yaw_int+=y_angle
        targetPos=-(Kp_p*y_angle+Kd_p*y_angular)
        motorError=targetPos-motorPos
        motorErrorI+=motorError
        motorErrorD=motorError-lastMotorError
        lastMotorError=motorError
        print("P:",motorError)
        print("I:",motorErrorI)
        print("D:",motorErrorD)
        pub_back.publish(-(Kp_p*(y_angle) + Ki_p*yaw_int + Kd_p*y_angular))
        # pub_back.publish(-(Kp_p*(y_angle-y_reference) + Kd_p*((y_angle-y_reference)-lastD)))
        # pub_back.publish(-(Kp_p*y_angle_imu + Kd_p*y_angular_imu))
        # pub_back.publish(pid(y_angle))
        time2 = time.time()
        interval = time2 - time1
        if(interval < time_interval):
            time.sleep(time_interval - interval)

if __name__ == '__main__':
    rospy.init_node('commander', anonymous=True)
    rospy.Subscriber("/gazebo/link_states", LinkStates, get_cart_pose)
    rospy.Subscriber("/imu", Imu, imucallback)
    rospy.Subscriber("/refIn", Float64, refCallback)
    rospy.Subscriber("/Kp", Float64, KpCallback)
    rospy.Subscriber("/Ki", Float64, KiCallback)
    rospy.Subscriber("/Kd", Float64, KdCallback)
    rospy.Subscriber("/joint_states", JointState, get_joint)
    commander()
    rospy.spin()
