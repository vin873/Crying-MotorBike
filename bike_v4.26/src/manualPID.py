#! /usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import String

command = ''

pubP = rospy.Publisher('/Kp', Float64, queue_size = 10)
pubI = rospy.Publisher('/Ki', Float64, queue_size = 10)
pubD = rospy.Publisher('/Kd', Float64, queue_size = 10)
pubGo = rospy.Publisher('/back_wh_position_controller/command' , Float64, queue_size = 10)

def publisher():
    rospy.init_node("aaaaaaaa")

    coolFloat = Float64()

    while not rospy.is_shutdown():
        command = input("P, I or D : ")
        if command == 'P' or command == 'p' or command == 'I' or command == 'i' or command == 'D' or command == 'd' or command == 'G' or command == 'g':
            value = input("Value : ")
            coolFloat.data=float(value)
            if command == 'P' or command == 'p':
                pubP.publish(coolFloat)
            if command == 'I' or command == 'i':
                pubI.publish(coolFloat)
            if command == 'D' or command == 'd':
                pubD.publish(coolFloat)
            if command == 'G' or command == 'g':
                pubGo.publish(coolFloat)
            
        else:
            print("Input error !!\n")
            print("======================================== \n")
            continue
        rospy.sleep(1)

if __name__=="__main__":
    try:
        publisher()

    except rospy.ROSInterruptException:
        pass