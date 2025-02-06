#!/usr/bin/env python

import rospy
import numpy as np
from numpy.linalg import inv
import time
import csv
import matplotlib.pyplot as plt
from std_msgs.msg import String
from whole_body_state_msgs.msg import JointState, WholeBodyTrajectory, WholeBodyState, RhonnState


def msg_callback(data):
    global u, received_data, time_sim, flag_new_message, time_sim, time_now

    rospy.loginfo('Start of  motor callback')

    time_now = data.time

    if time_sim == time_now:
        flag_new_message = True

    u = data.joints[0].effort

    rospy.loginfo('End of motor callback')



def my_node():
    global u, received_data, time_sim, flag_new_message, time_sim, time_now

    time_sim = 0.0
    time_now = 0.0
    dt = 0.001
    duration = 10
    position_future = 0.0
    position_now = 0.0
    velocity_future = 0.0
    velocity_now = 0.0
    u = 0.0
    received_data = None
    b = 0.03 # 0.0167
    J = 0.031 # 0.0167
    flag_new_message = False

    rospy.init_node('motor_dc', anonymous=True)

    topic_name = "/robot_states"

    pub = rospy.Publisher(topic_name, WholeBodyState, queue_size=1)

    rospy.Subscriber('/reemc/efforts', WholeBodyState, msg_callback, queue_size=1)
    
    rate = rospy.Rate(100)

    # Node verifaction
    rospy.loginfo("Node is running. Listening on /reemc/efforts and publishing to /robot_states...")
    
    rospy.loginfo("Motor working...")

    while not rospy.is_shutdown():


        name = 'right_joint_1'

        if flag_new_message == True:

            flag_new_message = False
            position_future = position_now + (dt*velocity_now)
            velocity_future = velocity_now + dt*(-((b*velocity_now)/J) + (u/J))
            time_sim = time_sim + dt

        rospy.loginfo("Time passed {} motor".format(time_sim))

        #Control Publisher
        position_msg = WholeBodyState()
        joint_estimation = JointState()

        joint_estimation.name = name
        joint_estimation.position = position_future
        joint_estimation.velocity = velocity_future
        joint_estimation.effort = u

        position_msg.joints.append(joint_estimation)
        position_msg.time = time_sim
        pub.publish(position_msg)

        position_now = position_future
        velocity_now = velocity_future

        if time_sim >= duration:
            rospy.signal_shutdown("Time over ...")

        

        # if received_data is not None:
        #     rospy.loginfo("Processing subscriber data: %s", received_data)
        #     # Reset received data (optional)
        #     received_data = None


        rate.sleep()
    rospy.loginfo("Motor stopped...")


if __name__ == '__main__':
    try:
        my_node()
    except rospy.ROSInterruptException:
        #ending_node()
        pass