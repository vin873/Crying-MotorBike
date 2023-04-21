#!/usr/bin/env python3
import rospy, time
from std_msgs.msg import Float64
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose, Twist
from std_srvs.srv import Empty
import math
state = Pose()
target_yaw_angle = 0
last_error_pos = 0
integral_yaw_error = 0
yaw_displacement_sum = 0
y_angle = 0

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    return roll_x # in radians

def get_pose(data):
    global y_angle, state
    ind_pitch = data.name.index('Wheel::base_link')
    state = data.pose[ind_pitch]
    y_angle = euler_from_quaternion(state.orientation.x, state.orientation.y, state.orientation.z, state.orientation.w)

def simPid():
    global y_angle, target_yaw_angle, state, last_error_pos, integral_yaw_error, yaw_displacement_sum, y_angle
    Kp_p = -7.423594013529117
    Ki_p = -123.285936099733413/10
    Kd_p = -0.4656542182778076/10
    time_interval = 0.001
    time1 = time.time()

    error_pos = target_yaw_angle - y_angle
    integral_yaw_error += (error_pos + last_error_pos)*time_interval/2
    effort_pos = -(Kp_p*error_pos  + 
                    Ki_p*integral_yaw_error +
                    Kd_p*(error_pos - last_error_pos)/time_interval)   
    last_error_pos = error_pos

    pub_back.publish(effort_pos)
    # pub_front.publish(effort_pos2)
    time2 = time.time()
    interval = time2 - time1
    if(interval < time_interval):
        time.sleep(time_interval - interval)

def listener():
    rospy.init_node("runner")
    pub_back = rospy.Publisher('/Revolute_2_position_controller/command', Float64, queue_size = 100)
    rospy.Subscriber("/gazebo/link_states", LinkStates, get_pose)
    rate=rospy.Rate(2000)
    while not rospy.is_shutdown():
        pub_back.publish(-y_angle)
        rate.sleep()

if __name__=="__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        pass