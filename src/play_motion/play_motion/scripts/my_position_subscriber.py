#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

def callback(data):
    print("Hello World")

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('joint_states', JointState, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
