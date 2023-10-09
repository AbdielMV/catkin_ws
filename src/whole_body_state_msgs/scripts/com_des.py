#!/usr/bin/env python
import os
import unittest
from math import sqrt

import sys

import numpy
sys.path.append("/home/niger/curves/build/python/curves")

import eigenpy
import numpy as np
from numpy import array, array_equal, isclose, random, zeros
from numpy.linalg import norm
import pickle

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

from curves import exact_cubic, curve_constraints

import rospy
import pinocchio
from std_msgs.msg import String
from whole_body_state_msgs.whole_body_state_publisher import WholeBodyStatePublisher
from whole_body_state_msgs.msg import WholeBodyState

from geometry_msgs.msg import Accel

import numpy as np

# This is from footstep planner
curva={}
curva[1]=[[0,-0.1,0],[0.16,-0.1,0.08],[0.32,-0.1,0]]
curva[2]=[[0,0.1,0],[0.34,0.1,0.08],[0.68,0.1,0]]
curva[3]=[curva[1][2],[curva[2][2][0],-0.1,0.08],[1.04,-0.1,0]]
curva[4]=[curva[2][2],[curva[3][2][0],0.1,0.08],[1.400,0.1,0]]
curva[5]=[curva[3][2],[curva[4][2][0],-0.1,0.08],[1.76,-0.1,0]]
curva[6]=[curva[4][2],[curva[5][2][0],0.1,0.08],[2.12,0.1,0]]
curva[7]=[curva[5][2],[curva[6][2][0],-0.1,0.08],[2.48,-0.1,0]]
curva[8]=[curva[6][2],[curva[7][2][0],0.1,0.08],[2.76,0.1,0]]
curva[9]=[curva[7][2],[curva[8][2][0],-0.1,0.08],[3,-0.1,0]]
curva[10]=[curva[8][2],[2.88,0.1,0.08],[3,0.1,0]]

T0=0.0
T1=0.4
T2=0.8

# These come from a footstep planner
p10 = curva[9][2]
p9 = curva[8][2]
p8 = curva[7][2]
p7 = curva[6][2]
p6 = curva[5][2]
p5 = curva[4][2]
p4 = curva[3][2]
p3 = curva[4][0]
p2 = curva[3][0]
p1 = curva[2][0]


g = 9.81 
h = 0.7 # Desired height
w = np.sqrt(g/h) # Eigenfrequency of the pendulum

# CoP's of footsteps
p = dict()
p[1] = p1
p[2] = p2
p[3] = p3
p[4] = p4
p[5] = p5
p[6] = p6
p[7] = p7
p[8] = p8
p[9] = p9
p[10] = p10

icp_eos = dict() # eos = end of step
icp_ini = dict() # ini = inicial of step

n = 9 # Number of steps
icp_eos[n] = p[10]

# Backward calculation
for i in range(n,0,-1):
    icp_ini[i] = np.zeros(3)
    for j in range(3):
        icp_ini[i][j] = p[i][j] + 1/np.exp(T1*w) * (icp_eos[i][j] - p[i][j])      
    icp_eos[i-1] = icp_ini[i]

urdf_filename = '/home/niger/reemc_public_ws/src/whole_body_state_msgs/urdf/reemc_full_ft_hey5.urdf'
free_flyer = pinocchio.JointModelFreeFlyer()

topic = 'robot_states'
model = pinocchio.buildModelFromUrdf(urdf_filename, free_flyer)
data = model.createData()

q = np.zeros(model.nq)
v = np.zeros(model.nv)
icp = np.zeros(2)
com = np.zeros(3)
tau = np.zeros(model.njoints - 2)

def robot_states(msg):

    q[3] = msg.centroidal.base_orientation.x
    q[4] = msg.centroidal.base_orientation.y
    q[5] = msg.centroidal.base_orientation.z
    q[6] = msg.centroidal.base_orientation.w
    v[3] = msg.centroidal.base_angular_velocity.x
    v[4] = msg.centroidal.base_angular_velocity.y
    v[5] = msg.centroidal.base_angular_velocity.z
    
    for j in range(len(msg.joints)):
        jointId = model.getJointId(msg.joints[j].name) - 2
        q[jointId + 7] = msg.joints[j].position
        v[jointId + 6] = msg.joints[j].velocity
        tau[jointId] = msg.joints[j].effort

    pinocchio.centerOfMass(model, data, q, v)
    q[0] = msg.centroidal.com_position.x - data.com[0][0]
    q[1] = msg.centroidal.com_position.y - data.com[0][1]
    q[2] = msg.centroidal.com_position.z - data.com[0][2]
    v[0] = msg.centroidal.com_velocity.x - data.vcom[0][0]
    v[1] = msg.centroidal.com_velocity.y - data.vcom[0][1]
    v[2] = msg.centroidal.com_velocity.z - data.vcom[0][2]
    icp[0] = data.com[0][0] + 1/w * data.vcom[0][0]
    icp[1] = data.com[0][1] + 1/w * data.vcom[0][1]
    com = data.com[0]
#dA = numpy.zeros((6,36))
pinocchio.computeCentroidalMapTimeVariation(model, data, q, v)
dAv = data.dAg.dot(v)

def talker():
   rospy.init_node('com_des') #anonymous=True
   pub = rospy.Publisher('chatter', String, queue_size=10)
   com_des_pub = rospy.Publisher('com_des', Accel, queue_size=10)
   rospy.Subscriber("robot_states", WholeBodyState, robot_states)
   
   rate = rospy.Rate(10) # 10hz
   k_xi = 3

   icp_ref = dict()
   steptime = 10
   t = np.linspace(T0, T1, steptime) 
   for i in range(1,n+1,1):
      icp_ref[i] = dict()
      for j in range(3):
         icp_ref[i][j] = np.zeros(steptime)
         for k in range(steptime): # k can be seen as the varying time
            icp_ref[i][j][k] = p[i][j] + np.exp(t[k]*w) * (icp_ini[i][j] - p[i][j]) 

   while not rospy.is_shutdown():
      hello_str = "hello world %s" % rospy.get_time()
      rospy.loginfo(hello_str)
      pub.publish(hello_str)
      #wbs.publish(t, q, v, tau, p, pd, f, s)
      com_des_task = Accel()
      # ICP computation
      for i in range(1,n+1,1):
         for k in range(steptime): # k can be seen as the varying time
            p_des = [p[i][0] + (1 + k_xi/w) * (icp[0] -icp_ref[i][0][k]), p[i][1] + (1 + k_xi/w) * (icp[1] -icp_ref[i][1][k])]
            com_des_task.linear.x = g/h * (com[0]-p_des[0])-dAv[0]
            com_des_task.linear.y =  g/h * (com[1]-p_des[1])-dAv[1]
            com_des_task.linear.z =  0
            com_des_pub.publish(com_des_task)
            rate.sleep()

if __name__ == '__main__':
   try:
      talker()
   except rospy.ROSInterruptException:
      pass
      
