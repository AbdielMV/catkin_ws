#!/usr/bin/env python

import rospy
import numpy as np
from numpy.linalg import inv
import time
import csv
import matplotlib.pyplot as plt
from std_msgs.msg import String
from whole_body_state_msgs.msg import JointState, WholeBodyTrajectory, WholeBodyState, RhonnState

def rhonn_states_callback(data, joint_index):
    global position_rhonn,velocity_rhonn, w11, w12, w21, w22

    # Extract joint-specific information
    position_rhonn = data.rhonn[joint_index].position
    velocity_rhonn = data.rhonn[joint_index].velocity
    w11 = data.rhonn[joint_index].w11
    w12 = data.rhonn[joint_index].w12
    w21 = data.rhonn[joint_index].w21
    w22 = data.rhonn[joint_index].w22

def robot_states_callback(data, joint_index):
    global name,position,velocity,time_now

    # Extract joint-specific information
    name = data.joints[joint_index].name
    position = data.joints[joint_index].position
    velocity = data.joints[joint_index].velocity
    time_now = data.time


def sgm(state):
    return np.tanh(state)

def tetha_function(t,tetha_start,tetha_end,T):
    tetha = tetha_start + (((3*t**2)/T**2)-((2*t**3)/T**3))*(tetha_end - tetha_start)
    return tetha

""" def plot_data(count,time_series,sensor,rhonn,ref,error,ref2,z1,z2,u):

    # Close any previously opened plots
    plt.close('all')

    print('Plotting...')

    plt.figure(1,figsize=(12, 6))

    plt.subplot(2,1,1)
    plt.plot(time_series[0,:count],np.rad2deg(sensor[0,:]), label='Position')
    plt.plot(time_series[0,:count],np.rad2deg(rhonn[0,:count]), label='RHONN')
    # plt.plot(time_series[0,:count],np.rad2deg(ref[0,:count]), label='Reference')
    plt.xlabel('Time (s)')
    plt.ylabel('Position')
    plt.title('Position vs. Time')
    plt.legend(loc='best')
    plt.grid()

    plt.subplot(2,1,2)
    plt.plot(time_series[0,:count],np.rad2deg(sensor[1,:]), label='Velocity')
    plt.plot(time_series[0,:count],np.rad2deg(rhonn[1,:count]), label='RHONN')
    # plt.plot(time_series[0,:count],np.rad2deg(ref2[0,:count]), label='X1d')
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

    plt.figure(3,figsize=(12,6))
    plt.plot(time_series[0,:count],u[0,:count], label='Torque')
    plt.xlabel('Time (s)')
    plt.ylabel('Torque')
    plt.title('Control Law')
    plt.legend(loc='best')
    plt.grid()
    plt.figure(4,figsize=(12, 6))

    plt.subplot(2,1,1)
    plt.plot(time_series[0,:count],np.rad2deg(z1[0,:count]), label='Error 1')
    plt.xlabel('Time (s)')
    plt.ylabel('Value')
    plt.title('Tracking Error')
    plt.legend(loc='best')
    plt.grid()

    plt.subplot(2,1,2)
    plt.plot(time_series[0,:count],np.rad2deg(z2[0,:count]), label='Error 2')
    plt.xlabel('Time (s)')
    plt.ylabel('Value')
    plt.title('Tracking Error')
    plt.legend(loc='best')
    plt.grid()

    plt.show() """

def my_node():

    global position, velocity, position_rhonn, velocity_rhonn, effort, time_now, k, w11, w12, w21, w22

    dt = 0.001 #0.0025
    k = 0
    duration = 50
    samples = int(duration/dt) + 1
    X = np.zeros((2,samples))
    XN = np.zeros((2,samples + 1))
    u = np.zeros((1,samples + 1))
    time_sim = np.zeros((1,samples + 2))
    time_planner = np.ones((1,samples + 2))
    time_now = 0.0
    y = np.zeros((1,samples + 2))
    z0 = np.zeros((1,samples + 1))
    z1 = np.zeros((1,samples + 1))
    s = np.ones((1,samples))
    alpha = np.ones((1,samples))
    v = np.ones((1,samples))
    x1d = np.zeros((1,samples + 1))
    name = ""
    position = 0.0
    velocity = 0.0
    effort = 0.0
    position_rhonn = 0.0
    velocity_rhonn = 0.0
    w11 = 0.0
    w12 = 0.0
    w13 = 1
    w21 = 0.0
    w22 = 0.0
    w23 = 1


    rospy.init_node('identifier', anonymous=True)

    joint_index = rospy.get_param("~joint_index", 21)  # Default to 21

    # topic_name = "/efforts/joint_{}".format(joint_index)
    topic_name = "/reemc/efforts"

    pub = rospy.Publisher(topic_name, WholeBodyState, queue_size=1)

    # Use lambda to pass pub and joint_index to the callback
    rospy.Subscriber('/reemc/rhonn', WholeBodyState, lambda data: rhonn_states_callback(data, joint_index), queue_size=1)

    rospy.Subscriber('/robot_states', WholeBodyState, lambda data: robot_states_callback(data, joint_index), queue_size=1)

    rate = rospy.Rate(100) 

    while not rospy.is_shutdown():

        if time_sim[0,k] != time_now:
            k = k + 1
        else:
            k = k 

        rospy.loginfo("Time passed {} control".format(time_sim[0,k]))

        if time_sim[0,k] > duration:
            rospy.signal_shutdown("Time over ...")
        
        time_sim[0,k] = time_now
        time_sim[0,k+1] = time_sim[0,k] + dt
        time_sim[0,k+2] = time_sim[0,k] + (2*dt)

        #RHONN
        X[0,k] = position
        X[1,k] = velocity
        XN[0,k] = position_rhonn
        XN[1,k] = velocity_rhonn

        #Planner

        """ tetha_inicial = 0
        tetha_final = np.deg2rad(50)
        T = 1
        x_delay = 1
        time_planner[0,k] = time_sim[0,k] - x_delay
        time_planner[0,k+1] = time_sim[0,k+1] - x_delay
        time_planner[0,k+2] = time_sim[0,k+2] - x_delay

        if time_sim[0,k] < 1:
            y[0,k] = 0
            y[0,k+1] = 0
            y[0,k+2] = 0
        elif time_sim[0,k] >= 1 and time_sim[0,k] <= T+x_delay:
            y[0,k] = tetha_function(time_planner[0,k],tetha_inicial,tetha_final,T)
            y[0,k+1] = tetha_function(time_planner[0,k+1],tetha_inicial,tetha_final,T)
            y[0,k+2] = tetha_function(time_planner[0,k+2],tetha_inicial,tetha_final,T)
        elif time_sim[0,k] > T+x_delay:
            y[0,k] = tetha_final
            y[0,k+1] = tetha_final
            y[0,k+2] = tetha_final """
        
        tetha_inicial = 0
        tetha_final = np.deg2rad(50)
        T = 2
        time_planner[0,k] = time_sim[0,k]
        time_planner[0,k+1] = time_sim[0,k+1]
        time_planner[0,k+2] = time_sim[0,k+2]

        if time_sim[0,k] <= T:
            y[0,k] = tetha_function(time_planner[0,k],tetha_inicial,tetha_final,T)
            y[0,k+1] = tetha_function(time_planner[0,k+1],tetha_inicial,tetha_final,T)
            y[0,k+2] = tetha_function(time_planner[0,k+2],tetha_inicial,tetha_final,T)
        elif time_sim[0,k] > T:
            y[0,k] = tetha_final
            y[0,k+1] = tetha_final
            y[0,k+2] = tetha_final

        #Control

        # Gains Lineal Control
        k1 = 0.8 #-45
        k2 = 0.1 #-2
        k3 = 0
        u0 = 12

        # Gains Super-Twisting
        # k1 = -6  #-75  #-6
        # k2 = 0.7 #1.3  #0.7
        # k3 = 0 #0.1  #0.1

        #Block Control
        z0[0,k] = X[0,k] - y[0,k]

        z0[0,k+1] = XN[0,k] - y[0,k+1]

        x1d[0,k] = (1.0/w13)*(-(w11*sgm(X[0,k])) - w12 + y[0,k+1] + (k1*z0[0,k]))

        x1d[0,k+1] = (1.0/w13)*(-(w11*sgm(XN[0,k])) - w12 + y[0,k+2] + (k1*z0[0,k+1]))

        z1[0,k] = X[1,k] - x1d[0,k]
        
        z1[0,k+1] = XN[1,k] - x1d[0,k+1]

        # s[0,k] = z1[0,k+1]

        # alpha[0,k+1] = alpha[0,k] + (dt*(-k3*np.sign(s[0,k])))

        # v[0,k] = (-k2*np.sqrt(np.abs(s[0,k]))*np.sign(s[0,k])) + alpha[0,k]

        """ if time_sim[0,k] < 1:
            u[0,k+1] = 0.0
        else:
            #Lineal
            ueq = (1.0/w23)*( -(w21*sgm(X[0,k])) - (w22*sgm(X[1,k])) + x1d[0,k+1] + k2*z1[0,k])
            if np.abs(ueq) <= u0:
                u[0,k+1] = ueq 
            else:
                u[0,k+1] = (u0*ueq)/np.abs(ueq) """
        
        u[0,k+1] = (1.0/w23)*( -(w21*sgm(X[0,k])) - (w22*sgm(X[1,k])) + x1d[0,k+1] + (k2*z1[0,k]))

        
        # u[0,k+1] = 0.05*np.sin(3*time_sim[0,k])

        #Control Publisher
        position_msg = WholeBodyState()
        joint_estimation = JointState()
        rhonn_estimation = RhonnState()

        joint_estimation.name = name
        joint_estimation.effort = u[0,k+1]
        rhonn_estimation.z0 = z0[0,k]
        rhonn_estimation.z1 = z1[0,k]
        rhonn_estimation.reference = y[0,k]

        position_msg.joints.append(joint_estimation)
        position_msg.rhonn.append(rhonn_estimation)
        position_msg.header.stamp = rospy.Time.now()
        position_msg.time = time_sim[0,k]
        pub.publish(position_msg)
        

        #if k >= samples-2:
        """ if time_sim[0,k] >= (duration - dt):
            joint_estimation.name = name
            joint_estimation.effort = 0.15

            position_msg.joints.append(joint_estimation)
            position_msg.header.stamp = rospy.Time.now()
            position_msg.time = time_sim[0,k]
            pub.publish(position_msg)
            rospy.signal_shutdown("Time over ...") """     

        rate.sleep()

    rospy.loginfo("Ending program")

    # plot_data(samples,time_sim,X,XN,y,error,x1d,z0,z1,u)

if __name__ == '__main__':
    try:
        my_node()
    except rospy.ROSInterruptException:
        #ending_node()
        pass