#!/usr/bin/env python

import rospy
import os
import numpy as np
from numpy.linalg import inv
import time
import csv
import matplotlib.pyplot as plt
from std_msgs.msg import String
from whole_body_state_msgs.msg import JointState, WholeBodyTrajectory, WholeBodyState, RhonnState

def robot_states_callback(data, joint_index):
    global name, position, velocity, time_now

    # Extract joint-specific information
    name = data.joints[joint_index].name
    position = data.joints[joint_index].position
    velocity = data.joints[joint_index].velocity
    time_now = data.time
    
def control_states_callback(data, joint_index):
    global effort

    # Extract joint-specific information
    effort = data.joints[joint_index].effort


def sgm(state):
    return np.tanh(state)


""" def plot_data(count,time_series,sensor,rhonn,error):
    
    # Close any previously opened plots
    plt.close('all')

    print('Plotting...')

    plt.figure(1,figsize=(12, 6))

    plt.subplot(2,1,1)
    plt.plot(time_series[0,:count],np.rad2deg(sensor[0,:]), label='Position')
    plt.plot(time_series[0,:count],np.rad2deg(rhonn[0,:count]), label='RHONN')
    plt.xlabel('Time (s)')
    plt.ylabel('Position')
    plt.title('Position vs. Time')
    plt.legend(loc='best')
    plt.grid()

    plt.subplot(2,1,2)
    plt.plot(time_series[0,:count],np.rad2deg(sensor[1,:]), label='Velocity')
    plt.plot(time_series[0,:count],np.rad2deg(rhonn[1,:count]), label='RHONN')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity')
    plt.title('Velocity vs. Time')
    plt.legend(loc='best')
    plt.grid()

    plt.figure(2,figsize=(12, 6))

    plt.subplot(2,1,1)
    plt.plot(time_series[0,:count],np.rad2deg(error[0,:]), label='Error P')
    plt.xlabel('Time (s)')
    plt.ylabel('Degrees')
    plt.title('Estimation Error')
    plt.legend(loc='best')
    plt.grid()

    plt.subplot(2,1,2)
    plt.plot(time_series[0,:count],np.rad2deg(error[1,:]), label='Error V')
    plt.xlabel('Time (s)')
    plt.ylabel('Degrees')
    plt.title('Estimation Error')
    plt.legend(loc='best')
    plt.grid()

    plt.show() """


def my_node():

    global position, velocity, effort, time_now

    dt = 0.001 #0.0025
    k = 0
    duration = 50
    samples = int(duration/dt) + 1
    X = np.zeros((2,samples))
    XN = np.zeros((2,samples + 1))
    """ w1 = np.random.rand(2, samples + 1)
    w2 = np.random.rand(2, samples + 1) """
    w1 = np.zeros((2, samples + 1))
    w2 = np.zeros((2, samples + 1))
    error = np.zeros((2,samples))
    p1 = np.zeros((samples + 1,2,2))
    p2 = np.zeros((samples + 1,2,2))
    p1[0] = 1e10 * np.eye(2)
    p2[0] = 1e10 * np.eye(2)
    time_sim = np.zeros((1,samples + 2))
    time_now = 0.0
    name = 'right_joint_1'
    position = 0.0
    velocity = 0.0
    effort = 0.0
    position_y1 = 0.0
    velocity_y1 = 0.0
    position_1 = 0.0
    velocity_1 = 0.0

    rospy.init_node('identifier', anonymous=True)

    joint_index = rospy.get_param("~joint_index", 21)  # Default to 21

    # topic_name = "/efforts/joint_{}".format(joint_index)
    topic_name = "/reemc/rhonn"

    pub = rospy.Publisher(topic_name, WholeBodyState, queue_size=1)

    # Use lambda to pass pub and joint_index to the callback
    rospy.Subscriber('/robot_states', WholeBodyState, lambda data: robot_states_callback(data, joint_index), queue_size=1)
    rospy.Subscriber('/reemc/efforts', WholeBodyState, lambda data: control_states_callback(data, joint_index), queue_size=1)

    rate = rospy.Rate(100) 

    while not rospy.is_shutdown():

        if time_sim[0,k] != time_now:
            k = k + 1
        else:
            k = k
        
        rospy.loginfo("Time passed {} ident".format(time_sim[0,k]))

        if time_sim[0,k] > duration:
            rospy.signal_shutdown("Time over ...")

        time_sim[0,k] = time_now        
        time_sim[0,k+1] = time_now + dt
        time_sim[0,k+2] = time_now + (2*dt)
        
        w13 = 1
        w23 = 1

        #First degree filter
        # position_y = position_y1*0.99+position_1*0.00995
        # velocity_y = velocity_y1*0.99+velocity_1*0.00995
        # position_1 = position
        # velocity_1 = velocity
        # position_y1 = position_y
        # velocity_y1 = velocity_y
        
        #Change of variable for use those names
        # position = position_y
        # velocity = velocity_y
        
        #RHONN
        X[0,k] = position
        X[1,k] = velocity

        C1 = np.array([[sgm(X[0, k])], [1]],dtype=float)
        C2 = np.array([[sgm(X[0, k])], [sgm(X[1, k])]],dtype=float)


        XN[0,k+1] = np.dot(w1[:,[k]].T,C1) + (w13*X[1,k])
        # XN[1,k+1] = np.dot(w2[:,[k]].T,C2) + (w23*u[0,k]*dt)
        XN[1,k+1] = np.dot(w2[:,[k]].T,C2) + (w23*effort*dt)
        
        #EFK
        dimH1 = C1.shape
        dimH2 = C2.shape

        H1 = C1
        H2 = C2
        eta = 0.6
        error[0,k] = X[0,k] - XN[0,k+1]
        error[1,k] = X[1,k] - XN[1,k+1]

        #Neuron 1
        R1 = 1e5
        Q1 = 1e5*np.eye(dimH1[0])
        M1 = 1.0/(R1 + np.dot(H1.T,np.dot(p1[k],H1)))
        #K1 equation
        K1 = np.dot(p1[k],np.dot(H1,M1))
        #w1 equation
        w1[:,[k+1]] = w1[:,[k]] + np.dot(eta,np.dot(K1,error[0,k]))
        #p1 equation
        p1[k+1] = p1[k] - np.dot(K1,np.dot(H1.T,p1[k])) + Q1

        
        #Neuron 2
        R2 = 1e8
        Q2 = 1e7*np.eye(dimH2[0])
        M2 = 1.0/(R2 + np.dot(H2.T,np.dot(p2[k],H2)))
        #K2 equation
        K2 = np.dot(p2[k],np.dot(H2,M2))
        #w2 equation
        w2[:,[k+1]] = w2[:,[k]] + np.dot(eta,np.dot(K2,error[1,k]))
        #p2 equation
        p2[k+1] = p2[k] - np.dot(K2,np.dot(H2.T,p2[k])) + Q2
        

        #Control Publisher
        position_msg = WholeBodyState()
        rhonn_estimation = RhonnState()

        rhonn_estimation.name = name
        rhonn_estimation.position = XN[0,k+1]
        rhonn_estimation.velocity = XN[1,k+1]
        rhonn_estimation.error_w1 = error[0,k]
        rhonn_estimation.error_w2 = error[1,k]
        rhonn_estimation.w11 = w1[0,[k]]
        rhonn_estimation.w12 = w1[1,[k]]
        rhonn_estimation.w21 = w2[0,[k]]
        rhonn_estimation.w22 = w2[1,[k]]

        position_msg.rhonn.append(rhonn_estimation)
        position_msg.header.stamp = rospy.Time.now()
        position_msg.time = time_sim[0,k]
        pub.publish(position_msg)
               

        rate.sleep()

    rospy.loginfo("Ending program")

    # plot_data(samples,time_sim,X,XN,error)

if __name__ == '__main__':
    try:
        my_node()
    except rospy.ROSInterruptException:
        #ending_node()
        pass