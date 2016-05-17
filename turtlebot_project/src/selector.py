#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
import os

def callback(data):
    if data.buttons[0] == 1:
        print os.system('rosnode kill /AR_tracker')
    elif data.buttons[8] == 1: 
        print os.system('rosnode kill -a')
    elif data.buttons[5] == 1: 
        print os.system('rosnode kill /color_tracker')
    
def listener():

    rospy.init_node('FunctionSelector', anonymous=True)

    rospy.Subscriber("joy", Joy, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
