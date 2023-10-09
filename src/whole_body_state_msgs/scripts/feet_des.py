#!/usr/bin/env python
import os
import unittest
from math import sqrt

import sys
sys.path.append("/home/niger/curves/build/python/curves")

import time
import eigenpy
import numpy as np
from numpy import array, array_equal, isclose, random, zeros
from numpy.linalg import norm
import pickle

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

from curves import exact_cubic, curve_constraints

#######################################################################################
############################ IMPORTAR PARA PUBLICAR ###################################
#######################################################################################
import rospy
#import pinocchio
from std_msgs.msg import String
from geometry_msgs.msg import Point
#from geometry_msgs.msg import Twist
#from geometry_msgs.msg import Accel
#from whole_body_state_msgs.whole_body_state_publisher import WholeBodyStatePublisher
#from whole_body_state_msgs.msg import WholeBodyState

#######################################################################################
#######################################################################################
#######################################################################################

curva={}
curva[1]=[[0.0,-0.1,0.0],[0.0,-0.1,0.20],[0.0,-0.1,0.40]]
curva[2]=[[0.0,0.1,0.0],[0.0,0.1,0.20],[0.0,0.1,0.40]]

#Creation of exact cubic
T0=0.0
T1=2.0
T2=4.0

waypoints={}
timewaypoints={}
ec={}
NumberOfSplines={}
FirstSpline={}

res={}

Tmincheck={}

c={}
ec2={}
steptime={}

x={}
y={}
z={}
t={}

x_vel={}
y_vel={}
z_vel={}

x_acc={}
y_acc={}
z_acc={}



for j in range(2):
    waypoints[j]=np.matrix(curva[j+1]).transpose()
    timewaypoints[j]=np.matrix([T0,T1,T2]).transpose()
    ec[j]=exact_cubic(waypoints[j],timewaypoints[j])
    NumberOfSplines[j]=ec[j].getNumberSplines()#Get number of splines
    FirstSpline[j]=ec[j].getSplineAt(0)#Get first spline (polynomial)

    #Evaluationatt=0.5
    res[j]=ec[j](0.5)

    #Derivativeorder1att=0.5
    res[j]=ec[j].derivate(0.5,1)

    #Upperandlowerboundofdefinitioninterval
    Tmincheck[j]=ec[j].min()
    Tmincheck[j]=ec[j].max()

    #Creation of exact cubic with constraints
    c[j]=curve_constraints(3)
    c[j].init_vel=np.matrix([0.0,0.,0.1]).transpose()
    c[j].end_vel=np.matrix([0.,0.,0.]).transpose()
    c[j].init_acc=np.matrix([0.0,0.,0.1]).transpose()
    c[j].end_acc=np.matrix([0.,0.,0.]).transpose()
    ec2[j]=exact_cubic(waypoints[j],timewaypoints[j],c[j])

    #Derivativeattimet
    res[j]=ec2[j].derivate(T0,1)#Equaltoinitvel
    res[j]=ec2[j].derivate(T0,2)#Equaltoinitacc
    res[j]=ec2[j].derivate(T2,1)#Equaltoendvel
    res[j]=ec2[j].derivate(T2,2)#Equaltoendacc

    steptime[j]=10

    x[j]=np.zeros(steptime[j])
    y[j]=np.zeros(steptime[j])
    z[j]=np.zeros(steptime[j])

    x_vel[j]=np.zeros(steptime[j])
    y_vel[j]=np.zeros(steptime[j])
    z_vel[j]=np.zeros(steptime[j])

    x_acc[j]=np.zeros(steptime[j])
    y_acc[j]=np.zeros(steptime[j])
    z_acc[j]=np.zeros(steptime[j])

    t[j]=np.linspace(T0, T2, steptime[j])


    for i in range(steptime[j]):
        x[j][i]=ec2[j](t[j][i])[0]
        y[j][i]=ec2[j](t[j][i])[1]
        z[j][i]=ec2[j](t[j][i])[2]

        x_vel[j][i]=ec2[j].derivate(t[j][i],1)[0]
        y_vel[j][i]=ec2[j].derivate(t[j][i],1)[1]
        z_vel[j][i]=ec2[j].derivate(t[j][i],1)[2]

        x_acc[j][i]=ec2[j].derivate(t[j][i],2)[0]
        y_acc[j][i]=ec2[j].derivate(t[j][i],2)[1]
        z_acc[j][i]=ec2[j].derivate(t[j][i],2)[2]

#print x
fig=plt.figure()
ax=plt.axes(projection ='3d')
plt.xlim(0,3)
plt.ylim(-0.3,0.3)
ax.set_zlim(0, 1)

ax.plot(x[0],y[0],z[0],label='1')
ax.plot(x[1],y[1],z[1],label='2')

ax.legend()
plt.show()


#######################################################################################
################################### CREAR PUBLISHER ###################################
#######################################################################################
def talker():
    rospy.init_node('talker', anonymous=True)
    pub_pos_left = rospy.Publisher('Position_left', Point, queue_size=10)
    pub_pos_right = rospy.Publisher('Position_right', Point, queue_size=10)
    rate = rospy.Rate(1)

    for h in range(10):
        p_pos_left=Point()
        p_pos_right=Point()
        
        p_pos_left.x=x[0][h]
        p_pos_left.y=y[0][h]
        p_pos_left.z=z[0][h]

        p_pos_right.x=x[1][0]
        p_pos_right.y=y[1][0]
        p_pos_right.z=z[1][0]

        pub_pos_left.publish(p_pos_left)
        pub_pos_right.publish(p_pos_right)
        rate.sleep()
        
    for i in range(10):
        p_pos_left=Point()
        p_pos_right=Point()
        
        p_pos_left.x=x[0][9]
        p_pos_left.y=y[0][9]
        p_pos_left.z=z[0][9]

        p_pos_right.x=x[1][i]
        p_pos_right.y=y[1][i]
        p_pos_right.z=z[1][i]

        pub_pos_left.publish(p_pos_left)
        pub_pos_right.publish(p_pos_right)
        rate.sleep()

    while not rospy.is_shutdown():

        p_pos_left=Point()
        p_pos_right=Point()
        
        p_pos_left.x=x[0][9]
        p_pos_left.y=y[0][9]
        p_pos_left.z=z[0][9]

        p_pos_right.x=x[1][9]
        p_pos_right.y=y[1][9]
        p_pos_right.z=z[1][9]

        pub_pos_left.publish(p_pos_left)
        pub_pos_right.publish(p_pos_right)
        rate.sleep()
      
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

#######################################################################################
#######################################################################################
#######################################################################################


