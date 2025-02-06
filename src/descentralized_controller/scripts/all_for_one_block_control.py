#!/usr/bin/env python

import rospy
import numpy as np
from numpy.linalg import inv
import time
import csv
import matplotlib.pyplot as plt
from std_msgs.msg import String
from whole_body_state_msgs.msg import JointState, WholeBodyTrajectory, WholeBodyState, RhonnState

def msg_callback(data, joint_index):
    global name, position, velocity, effort, time_now
    global flag_new_values, time_sim_now

    rospy.loginfo('Start of  ident callback')

    time_now = data.time
    rospy.loginfo("time now {}".format(time_now))
    rospy.loginfo('time sim now {}'.format(time_sim_now))

    if time_sim_now != time_now:
        flag_new_values = True

    rospy.loginfo('Flag status callback {}'.format(flag_new_values))

    # Extract joint-specific information
    name = data.joints[joint_index].name
    position = data.joints[joint_index].position
    velocity = data.joints[joint_index].velocity
    effort = data.joints[joint_index].effort

    rospy.loginfo('End of ident callback')



def sgm(state):
    return np.tanh(state)

def tetha_function(t,tetha_start,tetha_end,T):
    tetha = tetha_start + (((3*t**2)/T**2)-((2*t**3)/T**3))*(tetha_end - tetha_start)
    return tetha

def my_node():

    global position, velocity, effort, time_now
    global flag_new_values, time_sim_now

    dt = 0.001 #0.0025
    k = 0
    duration = 10
    samples = int(duration/dt) + 1
    X = np.zeros((2,samples))
    XN = np.zeros((2,samples + 1))
    # w1 = np.random.rand(2, samples + 1)
    # w2 = np.random.rand(2, samples + 1)
    w1 = np.zeros((2, samples + 1))
    w2 = np.zeros((2, samples + 1))
    u = np.zeros((1,samples + 1))
    error = np.zeros((2,samples))
    p1 = np.ones((samples + 1,2,2))
    p2 = np.ones((samples + 1,2,2))
    p1[0] = 1e10 * np.eye(2)
    p2[0] = 1e10 * np.eye(2)
    time_sim = np.zeros((1,samples + 2))
    time_sim_now = 0.0
    time_planner = np.zeros((1,samples + 2))
    time_now = 0.0
    y = np.zeros((1,samples + 2))
    z0 = np.zeros((1,samples + 1))
    z1 = np.zeros((1,samples + 1))
    s = np.zeros((1,samples))
    alpha = np.zeros((1,samples + 1))
    v = np.zeros((1,samples))
    x1d = np.zeros((1,samples + 1))
    name = ""
    position = 0.0
    velocity = 0.0
    effort = 0.0
    flag_new_values = False


    rospy.init_node('identifier', anonymous=True)

    joint_index = rospy.get_param("~joint_index", 0)  # Default to 21

    # topic_name = "/efforts/joint_{}".format(joint_index)
    topic_name = "/reemc/efforts"

    pub = rospy.Publisher(topic_name, WholeBodyState, queue_size=1)

    # Use lambda to pass pub and joint_index to the callback
    rospy.Subscriber('/robot_states', WholeBodyState, lambda data: msg_callback(data, joint_index), queue_size=1)

    rate = rospy.Rate(1000) 

    while not rospy.is_shutdown():
        rospy.loginfo('Start of while')

        rospy.loginfo('flag status {}'.format(flag_new_values))

        if flag_new_values == True:

            rospy.loginfo('Update values')

            flag_new_values = False

            k = k + 1

            time_sim[0,k] = time_now
            time_sim[0,k+1] = time_now + dt
            time_sim[0,k+2] = time_now + (2*dt)
            

            X[0,k] = position
            X[1,k] = velocity

            #End if
                
        rospy.loginfo("Time passed {} k".format(time_sim[0,k]))
        time_sim_now = time_sim[0,k]
        rospy.loginfo("k sample {} k".format(k))

        if time_sim[0,k] >= duration:
            rospy.signal_shutdown("Time over ...")

        
        w13 = 1
        w23 = 1
        
        #RHONN

        C1 = np.array([[sgm(X[0, k])], [1]],dtype=float)
        C2 = np.array([[sgm(X[0, k])], [sgm(X[1, k])]],dtype=float)


        XN[0,k+1] = np.dot(w1[:,[k]].T,C1) + (w13*X[1,k])
        XN[1,k+1] = np.dot(w2[:,[k]].T,C2) + (w23*u[0,k]*dt)
        # XN[1,k+1] = np.dot(w2[:,[k]].T,C2) + (w23*effort*dt)
        
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


        #Planner

        tetha_inicial = 0
        tetha_final = np.deg2rad(50)
        T = 2
        x_delay = 0
        time_planner[0,k] = time_sim[0,k] - x_delay
        time_planner[0,k+1] = time_sim[0,k+1] - x_delay
        time_planner[0,k+2] = time_sim[0,k+2] - x_delay

        """ if time_sim[0,k] < 1:
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
        
        if time_sim[0,k] < T :
            y[0,k] = tetha_function(time_planner[0,k],tetha_inicial,tetha_final,T)
            y[0,k+1] = tetha_function(time_planner[0,k+1],tetha_inicial,tetha_final,T)
            y[0,k+2] = tetha_function(time_planner[0,k+2],tetha_inicial,tetha_final,T)
        elif time_sim[0,k] > T:
            y[0,k] = tetha_final
            y[0,k+1] = tetha_final
            y[0,k+2] = tetha_final

        #Control

        # Gains Lineal Control
        k1 = 0.1 #-45
        k2 = 0.1 #-2
        k3 = 0.01
        # u0 = 12

        # Gains Super-Twisting
        # k1 = -6  #-75  #-6
        # k2 = 0.7 #1.3  #0.7
        # k3 = 0 #0.1  #0.1

        #Block Control
        z0[0,k] = X[0,k] - y[0,k]

        z0[0,k+1] = XN[0,k+1] - y[0,k+1]

        x1d[0,k] = (1.0/w13)*(-(w1[0,k]*sgm(X[0,k])) - w1[1,k] + y[0,k+1] + (k1*z0[0,k]))

        x1d[0,k+1] = (1.0/w13)*(-(w1[0,k]*sgm(XN[0,k+1])) - w1[1,k] + y[0,k+2] + (k1*z0[0,k+1]))

        z1[0,k] = X[1,k] - x1d[0,k]
        
        z1[0,k+1] = XN[1,k+1] - x1d[0,k+1]

        s[0,k] = z1[0,k+1]

        alpha[0,k+1] = alpha[0,k] + (dt*(-k3*np.sign(s[0,k])))

        v[0,k] = (-k2*np.sqrt(np.abs(s[0,k]))*np.sign(s[0,k])) + alpha[0,k]
        
        u[0,k+1] = (1.0/w23)*( -(w2[0,k]*sgm(X[0,k])) - (w2[1,k]*sgm(X[1,k])) + x1d[0,k+1] + k2*z1[0,k])

        # u[0,k+1] = (1.0/w23)*( -(w2[0,k]*sgm(X[0,k])) - (w2[1,k]*sgm(X[1,k])) + x1d[0,k+1] + v[0,k])


        # u[0,k+1] = 0.05*np.sin(3*time_sim[0,k])

        #Control Publisher
        position_msg = WholeBodyState()
        joint_estimation = JointState()
        rhonn_estimation = RhonnState()

        joint_estimation.name = name
        joint_estimation.effort = u[0,k+1]

        rhonn_estimation.name = name
        rhonn_estimation.position = XN[0,k+1]
        rhonn_estimation.velocity = XN[1,k+1]
        rhonn_estimation.error_w1 = error[0,k]
        rhonn_estimation.error_w2 = error[1,k]
        rhonn_estimation.w11 = w1[0,[k]]
        rhonn_estimation.w12 = w1[1,[k]]
        rhonn_estimation.w21 = w2[0,[k]]
        rhonn_estimation.w22 = w2[1,[k]]

        rhonn_estimation.z0 = z0[0,k]
        rhonn_estimation.z1 = z1[0,k]
        rhonn_estimation.reference = y[0,k]

        position_msg.joints.append(joint_estimation)
        position_msg.rhonn.append(rhonn_estimation)
        position_msg.header.stamp = rospy.Time.now()
        position_msg.time = time_sim[0,k]
        pub.publish(position_msg)

        flag_new_values = False


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
        rospy.loginfo('End of while')


    rospy.loginfo("Ending program")

    # plot_data(samples,time_sim,X,XN,y,error,x1d,z0,z1,u)

if __name__ == '__main__':
    try:
        my_node()
    except rospy.ROSInterruptException:
        #ending_node()
        pass