#!/usr/bin/env python

import rospy
import numpy as np
from numpy.linalg import inv
import time
import csv
import matplotlib.pyplot as plt
from std_msgs.msg import String
from whole_body_state_msgs.msg import JointState, WholeBodyTrajectory, WholeBodyState, RhonnState

dt = 0.001 #0.0025

#Gains
f11 = 8 #15
f12 = 0.01 #0.08
f21 = 1 #0.08
f22 = 8 #8

tetha_inicial = 0
tetha_final = np.deg2rad(40)
T = 2
k = 0
time_now = 0
time_past = 0
duration = 3
samples = duration/dt
X = np.zeros((2,samples))
XN = np.zeros((2,samples+1))
w1 = np.zeros((2,samples+1))
w2 = np.zeros((2,samples+1))
u = np.ones((1,samples+1))
error = np.zeros((2,samples))
p1 = np.zeros((samples+1,2,2))
p2 = np.zeros((samples+1,2,2))
p1[0] = 1e10 * np.eye(2)
p2[0] = 1e10 * np.eye(2)
time_sim = np.zeros((1,samples+2))
y = np.ones((1,samples+2))
z0 = np.zeros((1,samples+1))
z1 = np.zeros((1,samples+1))
s = np.zeros((1,samples+1))
alpha = np.zeros((1,samples+1))
v = np.zeros((1,samples+1))
x1d = np.zeros((1,samples+1))
name = ""
position = 0.0
velocity = 0.0
position_y1 = 0.0
velocity_y1 = 0.0
position_1 = 0.0
velocity_1 = 0.0
dw1 = np.zeros((1,samples+1))
X1des = np.zeros((1,samples+1))
dw2 = np.zeros((1,samples+1))
pub = rospy.Publisher('/reemc/efforts', WholeBodyState, queue_size=1)


def msg_callback(data): #dt = 0.01s

    global y,k,time_now,X,XN,w1,w2,dt,name,position,velocity
    global time_past,u,p1,p2,tetha_inicial,tetha_final,position_1,velocity_1
    global T,error,z0,z1,s,alpha,v,x1d,pub,position_y1,velocity_y1
    global dw1,X1des,dw2,f11,f12,f21,f22

    name = data.joints[21].name
    position = data.joints[21].position
    velocity = data.joints[21].velocity

    time_now = data.time

    print('time {}\n'.format(time_sim[0,k]))

    #Time simulation
    time_sim[0,k+1] = time_sim[0,k] + dt
    time_sim[0,k+2] = time_sim[0,k+1] + dt   
    
    w13 = 0.1
    w23 = 1

    #First degree filter
    position_y = position_y1*0.99+position_1*0.00995
    velocity_y = velocity_y1*0.99+velocity_1*0.00995
    position_1 = position
    velocity_1 = velocity
    position_y1 = position_y
    velocity_y1 = velocity_y
    #Change of variable for use those names
    position = position_y
    velocity = velocity_y
    
    #RHONN
    X[0,k] = position
    X[1,k] = velocity

    C1 = np.array([[sgm(X[0, k])], [1]],dtype=float)
    C2 = np.array([[sgm(X[0, k])], [sgm(X[1, k])]],dtype=float)


    XN[0,k+1] = np.dot(w1[:,[k]].T,C1) + (w13*X[1,k])
    XN[1,k+1] = np.dot(w2[:,[k]].T,C2) + (w23*u[0,k]*dt)
    
    #EFK
    dimH1 = C1.shape
    dimH2 = C2.shape

    H1 = C1
    H2 = C2
    eta = 0.5
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
    if time_sim[0,k] <= T:
        y[0,k] = tetha_function(time_sim[0,k])
        y[0,k+1] = tetha_function(time_sim[0,k+1])
        y[0,k+2] = tetha_function(time_sim[0,k+2])
    else:
        y[0,k] = tetha_final
        y[0,k+1] = tetha_final
        y[0,k+2] = tetha_final

    #Control

    #Super Twisting

    z0[0,k] = y[0,k] - X[0,k]

    dw1[0,k+1] = dw1[0,k] + (dt*np.sign(z0[0,k]))

    X1des[0,k] = ((1.0/w13)*(f11*np.sqrt(np.abs(z0[0,k]))*np.sign(z0[0,k]))) + (f12*dw1[0,k])

    z1[0,k] = X1des[0,k] - X[1,k]

    dw2[0,k+1] = dw2[0,k] + (dt*np.sign(z1[0,k]))

    u[0,k+1] = ((1.0/w23)*(f21*np.sqrt(np.abs(z1[0,k]))*np.sign(z1[0,k]))) + (f22*dw2[0,k])

    k += 1

    #Control Publisher
    position_msg = WholeBodyState()
    joint_estimation = JointState()

    joint_estimation.name = name
    joint_estimation.effort = u[0,k]

    position_msg.joints.append(joint_estimation)
    position_msg.header.stamp = rospy.Time.now()
    position_msg.time = rospy.get_time()
    pub.publish(position_msg)

    if k >= samples-2:
        joint_estimation.name = name
        joint_estimation.effort = 0.5

        position_msg.joints.append(joint_estimation)
        position_msg.header.stamp = rospy.Time.now()
        position_msg.time = rospy.get_time()
        pub.publish(position_msg)
        rospy.signal_shutdown('Time is over')



def my_node():

    rospy.init_node('identifier', anonymous=True)

    rospy.Subscriber('/robot_states', WholeBodyState, msg_callback, queue_size=1)

    # rospy.Timer(rospy.Duration(0.005),publisher_message)
   
    rospy.spin()

    plot_data(samples,time_sim,X,XN,y,error,x1d,z0,z1)

def sgm(state):
    return np.tanh(state)

def tetha_function(t):
    global tetha_inicial,tetha_final,T
    
    tetha = tetha_inicial + (((3*t**2)/T**2)-((2*t**3)/T**3))*(tetha_final - tetha_inicial)
    return tetha

def plot_data(count,time_series,sensor,rhonn,ref,error,ref2,z1,z2):

    # Close any previously opened plots
    plt.close('all')

    print('Plotting...')

    plt.figure(1,figsize=(12, 6))

    plt.subplot(2,1,1)
    plt.plot(time_series[0,:count],np.rad2deg(sensor[0,:]), label='Position')
    plt.plot(time_series[0,:count],np.rad2deg(rhonn[0,:count]), label='RHONN')
    plt.plot(time_series[0,:count],np.rad2deg(ref[0,:count]), label='Reference')
    plt.xlabel('Time (s)')
    plt.ylabel('Position')
    plt.title('Position vs. Time')
    plt.legend(loc='best')
    plt.grid()

    plt.subplot(2,1,2)
    plt.plot(time_series[0,:count],np.rad2deg(sensor[1,:]), label='Velocity')
    plt.plot(time_series[0,:count],np.rad2deg(rhonn[1,:count]), label='RHONN')
    #plt.plot(time_series[0,:count],np.rad2deg(ref2[0,:count]), label='X1d')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity')
    plt.title('Position vs. Time')
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

    plt.show()

if __name__ == '__main__':
    try:
        my_node()
    except rospy.ROSInterruptException:
        #ending_node()
        pass