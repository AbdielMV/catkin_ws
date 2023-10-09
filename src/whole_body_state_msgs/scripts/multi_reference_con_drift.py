#!/usr/bin/env python
import os
import unittest
from math import sqrt

import sys
sys.path.append("/home/niger/curves/build/python/curves")

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
import pinocchio
from std_msgs.msg import String
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Accel
from whole_body_state_msgs.whole_body_state_publisher import WholeBodyStatePublisher
from whole_body_state_msgs.msg import WholeBodyState

#######################################################################################
#######################################################################################
#######################################################################################

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

#Creation of exact cubic
T0=0.0
T1=1.0
T2=2.0

#######################################################################################
########################################ICP############################################
#######################################################################################
# # These come from a footstep planner
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
he = 0.7 # Desired height
w = np.sqrt(g/he) # Eigenfrequency of the pendulum

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

urdf_filename = '/home/niger/reemc_public_ws/src/reemc_simulation/reemc_gazebo/models/reemc_full/reemc_full_ft_hey5.urdf.urdf'
free_flyer = pinocchio.JointModelFreeFlyer()
topic = 'robot_states'
model = pinocchio.buildModelFromUrdf(urdf_filename, free_flyer)
data = model.createData()


q = np.zeros(model.nq)
v = np.zeros(model.nv)
icp = np.zeros(2)
com = np.zeros(3)
tau = np.zeros(model.njoints - 2)

##################################################

x_posicion=0
y_posicion=0
z_posicion=0

x_velocidad=0
y_velocidad=0
z_velocidad=0

x_aceleracion=0
y_aceleracion=0
z_aceleracion=0

Pkkr_right=[0,0,0]
Pkkr_left=[0,0,0]
Tkkkr_right=[0,0,0,0,0,0]
Tkkkr_left=[0,0,0,0,0,0]
Tkkkr_dot_right=[0,0,0,0,0,0]
Tkkkr_dot_left=[0,0,0,0,0,0]
d_right=[0,0,0,0,0,0]
d_left=[0,0,0,0,0,0]
Kp_matrix=np.identity(3)
Kd_matrix=Kd=np.identity(6)


##################################################

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

    global Pkkr_right
    global Pkkr_left
    global Tkkkr_right
    global Tkkkr_left
    global Tkkkr_dot_right
    global Tkkkr_dot_left
    global d_right
    global d_left
    global Kp_matrix
    global Kd_matrix
    
    for contact in msg.contacts:
        
        name = contact.name
        # Contact pose
        
        pose = contact.pose
        vel=contact.velocity

        position = np.array([pose.position.x, pose.position.y, pose.position.z])
        quaternion = pinocchio.Quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                                          pose.orientation.z)
        p[name] = pinocchio.SE3(quaternion, position)

        velocity = np.array([vel.linear.x, 
                           vel.linear.y, 
                           vel.linear.z, 
                           0, 0, 0])

        #print(name)
        #print(velocity)
 
        
        if((name=="right_sole_link") and (joint=="Right")):

            Pkkr_right=[pose.position.x-x_posicion,
                        pose.position.y-y_posicion,
                        pose.position.z-z_posicion
                        ]

            Tkkkr_right=[vel.linear.x-x_velocidad,
                         vel.linear.y-y_velocidad,
                         vel.linear.z-z_velocidad,
                         0,0,0]

            Tkkkr_dot_right=[x_aceleracion,
                             y_aceleracion,
                             z_aceleracion,
                             0,0,0]

            Tkkkr_left=[0,0,0,0,0,0]
            Tkkkr_dot_left=[0,0,0,0,0,0]



        if((name=="left_sole_link") and (joint=="Left")):
            
            Pkkr_left=[pose.position.x-x_posicion,
                       pose.position.y-y_posicion,
                       pose.position.z-z_posicion
                       ]

            Tkkkr_left=[vel.linear.x-x_velocidad,
                        vel.linear.y-y_velocidad,
                        vel.linear.z-z_velocidad,
                        0,0,0]

            Tkkkr_dot_left=[x_aceleracion,
                            y_aceleracion,
                            z_aceleracion,
                            0,0,0]

            Tkkkr_right=[0,0,0,0,0,0]
            Tkkkr_dot_right=[0,0,0,0,0,0]

    
    print("Posicion error:")
    print(Pkkr_right)
    print(Pkkr_left)
    print("Twist error:")
    print(Tkkkr_right)
    print(Tkkkr_left)
    print("Aceleracion:")
    print(Tkkkr_dot_right)
    print(Tkkkr_dot_left)





    Pkkr_right = np.array(Pkkr_right)
    Pkkr_left  = np.array(Pkkr_left)
    Pkkr_right = Kp_matrix.dot(Pkkr_right)
    Pkkr_left  = Kp_matrix.dot(Pkkr_left)

    Tkkkr_right = np.array(Tkkkr_right)
    Tkkkr_left  = np.array(Tkkkr_left)
    Tkkkr_right = Kd_matrix.dot(Tkkkr_right)
    Tkkkr_left  = Kd_matrix.dot(Tkkkr_left)

    Tkkkr_dot_right = np.array(Tkkkr_dot_right)
    Tkkkr_dot_left  = np.array(Tkkkr_dot_left)

    ############################################################################################################################################
    ######################################################## CONTROLADOR PD ####################################################################
    ############################################################################################################################################

    pinocchio.forwardKinematics(model,data,q,v,0*v)
    pinocchio.computeJointJacobians(model,data,q)
    pinocchio.framesForwardKinematics(model,data,q)

    #print(data.a[7])
    #print(data.a[7].linear)
    #print(data.a[7].linear[0])
   
    d_right[0] = Pkkr_right[0] + Tkkkr_right[0] + Tkkkr_dot_right[0] 
    d_right[1] = Pkkr_right[1] + Tkkkr_right[1] + Tkkkr_dot_right[1]
    d_right[2] = Pkkr_right[2] + Tkkkr_right[2] + Tkkkr_dot_right[2]
    d_right[3] =       0       + Tkkkr_right[3] + Tkkkr_dot_right[3]
    d_right[4] =       0       + Tkkkr_right[4] + Tkkkr_dot_right[4]
    d_right[5] =       0       + Tkkkr_right[5] + Tkkkr_dot_right[5]

    d_left[0] = Pkkr_left[0] + Tkkkr_left[0] + Tkkkr_dot_left[0]
    d_left[1] = Pkkr_left[1] + Tkkkr_left[1] + Tkkkr_dot_left[1]
    d_left[2] = Pkkr_left[2] + Tkkkr_left[2] + Tkkkr_dot_left[2]
    d_left[3] =       0      + Tkkkr_left[3] + Tkkkr_dot_left[3]
    d_left[4] =       0      + Tkkkr_left[4] + Tkkkr_dot_left[4]
    d_left[5] =       0      + Tkkkr_left[5] + Tkkkr_dot_left[5]

    print("Referencias PD:")
    print(d_right)
    print(d_left)

    #Esto es si es con data.a
    '''
    print("\nJ_dot*v right:")
    print(data.a[13])
    print("J_dot*v left:")
    print(data.a[7])
    
    d_right[0] = d_right[0] - data.a[13].linear[0]
    d_right[1] = d_right[1] - data.a[13].linear[1]
    d_right[2] = d_right[2] - data.a[13].linear[2]
    d_right[3] = d_right[3] - data.a[13].angular[0]
    d_right[4] = d_right[4] - data.a[13].angular[1]
    d_right[5] = d_right[5] - data.a[13].angular[2]

    d_left[0] = d_left[0] - data.a[7].linear[0]
    d_left[1] = d_left[1] - data.a[7].linear[1]
    d_left[2] = d_left[2] - data.a[7].linear[2]
    d_left[3] = d_left[3] - data.a[7].angular[0]
    d_left[4] = d_left[4] - data.a[7].angular[1]
    d_left[5] = d_left[5] - data.a[7].angular[2]
    '''
    

    #Esto es si es con getFrameClassicalAcceleration World
    '''
    a_right = pinocchio.getFrameClassicalAcceleration(model, data, 13,
							  pinocchio.ReferenceFrame.WORLD)
    a_left = pinocchio.getFrameClassicalAcceleration(model, data, 7,
                            pinocchio.ReferenceFrame.WORLD)

    print("\nJ_dot*v getFrameClassicalAcceleration right:")
    print(a_right)
    print("J_dot*v getFrameClassicalAcceleration left:")
    print(a_left)
    
    d_right[0] = d_right[0] - a_right.linear[0]
    d_right[1] = d_right[1] - a_right.linear[1]
    d_right[2] = d_right[2] - a_right.linear[2]
    d_right[3] = d_right[3] - a_right.angular[0]
    d_right[4] = d_right[4] - a_right.angular[1]
    d_right[5] = d_right[5] - a_right.angular[2]

    d_left[0] = d_left[0] - a_left.linear[0]
    d_left[1] = d_left[1] - a_left.linear[1]
    d_left[2] = d_left[2] - a_left.linear[2]
    d_left[3] = d_left[3] - a_left.angular[0]
    d_left[4] = d_left[4] - a_left.angular[1]
    d_left[5] = d_left[5] - a_left.angular[2]
    '''

    #Esto es si es con getFrameClassicalAcceleration LOCAL_WORLD_ALIGNED 	
    a_right_drift = pinocchio.getFrameClassicalAcceleration(model, data, 34,
							  pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    a_left_drift = pinocchio.getFrameClassicalAcceleration(model, data, 18,
                            pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)

    print("\nJ_dot*v getFrameClassicalAcceleration (drift) right:")
    print(a_right_drift)
    print("J_dot*v getFrameClassicalAcceleration (drift) left:")
    print(a_left_drift)

    d_right[0] = d_right[0] - a_right_drift.linear[0]
    d_right[1] = d_right[1] - a_right_drift.linear[1]
    d_right[2] = d_right[2] - a_right_drift.linear[2]
    d_right[3] = d_right[3] - a_right_drift.angular[0]
    d_right[4] = d_right[4] - a_right_drift.angular[1]
    d_right[5] = d_right[5] - a_right_drift.angular[2]

    d_left[0] = d_left[0] - a_left_drift.linear[0]
    d_left[1] = d_left[1] - a_left_drift.linear[1]
    d_left[2] = d_left[2] - a_left_drift.linear[2]
    d_left[3] = d_left[3] - a_left_drift.angular[0]
    d_left[4] = d_left[4] - a_left_drift.angular[1]
    d_left[5] = d_left[5] - a_left_drift.angular[2]

    print("Referencia Final:")
    print(d_right)
    print(d_left)

    print("--------------------------")
  
    

#dA = numpy.zeros((6,36))
pinocchio.computeCentroidalMapTimeVariation(model, data, q, v)
dAv = data.dAg.dot(v)

#######################################################################################
#######################################################################################
#######################################################################################


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

joint="Left"

numero=0

for j in range(10):
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
    c[j].init_vel=np.matrix([0.1,0.,0.1]).transpose()
    c[j].end_vel=np.matrix([0.,0.,0.]).transpose()
    c[j].init_acc=np.matrix([0.1,0.,0.1]).transpose()
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
ax.plot(x[2],y[2],z[2],label='3')
ax.plot(x[3],y[3],z[3],label='4')
ax.plot(x[4],y[4],z[4],label='5')
ax.plot(x[5],y[5],z[5],label='6')
ax.plot(x[6],y[6],z[6],label='7')
ax.plot(x[7],y[7],z[7],label='8')
ax.plot(x[8],y[8],z[8],label='9')
ax.plot(x[9],y[9],z[9],label='10')
ax.legend()
plt.show()




#######################################################################################
################################### CREAR PUBLISHER ###################################
#######################################################################################
def talker():
    rospy.init_node('talker', anonymous=True)
    pub_pos = rospy.Publisher('chatter_pos', Point, queue_size=10)
    pub_vel = rospy.Publisher('chatter_vel', Twist, queue_size=10)
    pub_acc = rospy.Publisher('chatter_acc', Accel, queue_size=10)
    pub_joint  = rospy.Publisher('chatter_joint', String, queue_size=10)
    com_des_pub = rospy.Publisher('com_des', Accel, queue_size=10)
    posture_des_pub = rospy.Publisher('posture_des', Accel, queue_size=10)

    rospy.Subscriber("robot_states", WholeBodyState, robot_states)
 
    rate = rospy.Rate(10) # 10hz

    contador = 1
    
    q_des = [0, 0, 0, 0, 0, 0]
    kp = 4
    kd = 6
    
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


    for g in range (10):
        for h in range (10):
            ###########################################################################
            #################### PUBLICA HASTA LLEGAR AL FINAL ########################
            ###########################################################################
            #while not rospy.is_shutdown():
    
                p_pos=Point()
                p_vel=Twist()
                p_acc=Accel()

                p_pos.x=x[g][h]
                p_pos.y=y[g][h]
                p_pos.z=z[g][h]
                
                p_vel.linear.x=x_vel[g][h]
                p_vel.linear.y=y_vel[g][h]
                p_vel.linear.z=z_vel[g][h]
                p_vel.angular.x=0
                p_vel.angular.y=0
                p_vel.angular.z=0

                p_acc.linear.x=x_acc[g][h]
                p_acc.linear.y=y_acc[g][h]
                p_acc.linear.z=z_acc[g][h]
                p_acc.angular.x=0
                p_acc.angular.y=0
                p_acc.angular.z=0

                ########################################################################################################
                ###################ESTO GENERA VARIABLES SOLAMENTE USADAS PARA MULTIREFERENCE###########################
                ########################################################################################################
                
                global x_posicion, y_posicion, z_posicion
                global x_velocidad, y_velocidad, z_velocidad
                global x_aceleracion, y_aceleracion, z_aceleracion

                x_posicion=x[g][h]
                y_posicion=y[g][h]
                z_posicion=z[g][h]

                x_velocidad=x_vel[g][h]
                y_velocidad=y_vel[g][h]
                z_velocidad=z_vel[g][h]

                x_aceleracion=x_acc[g][h]
                y_aceleracion=y_acc[g][h]
                z_aceleracion=z_acc[g][h]
                
                ########################################################################################################
                ########################################################################################################
                ########################################################################################################
                

                #while(pub_pos.get_num_connections() == 0):
                #    rate.sleep()

                pub_pos.publish(p_pos)
                pub_vel.publish(p_vel)
                pub_acc.publish(p_acc)

                equis="Coordenada x: %f" %p_pos.x
                ye   ="Coordenada y: %f" %p_pos.y
                zeta ="Coordenada z: %f \n" %p_pos.z

                equis_vel="Velocidad x: %f" %p_vel.linear.x
                ye_vel   ="Velocidad y: %f" %p_vel.linear.y
                zeta_vel ="Velocidad z: %f \n" %p_vel.linear.z

                equis_acc="Aceleracion x: %f" %p_acc.linear.x
                ye_acc   ="Aceleracion y: %f" %p_acc.linear.y
                zeta_acc ="Aceleracion z: %f \n" %p_acc.linear.z

                print(contador)
                
                global joint
            
                if((contador%10==1) and (joint=="Right")):
                    joint="Left"

                elif ((contador%10==1) and (joint=="Left")):
                    joint="Right"
                
                print(joint)
                pub_joint.publish(joint)
                '''
                rospy.loginfo(equis)
                rospy.loginfo(ye)
                rospy.loginfo(zeta)

                rospy.loginfo(equis_vel)
                rospy.loginfo(ye_vel)
                rospy.loginfo(zeta_vel)

                rospy.loginfo(equis_acc)
                rospy.loginfo(ye_acc)
                rospy.loginfo(zeta_acc)
                '''

                contador=contador+1

                com_des_task = Accel()
                if (g<9):
                    p_des = [p[g+1][0] + (1 + k_xi/w) * (icp[0] -icp_ref[g+1][0][h]), p[g+1][1] + (1 + k_xi/w) * (icp[1] -icp_ref[g+1][1][h])]
                    com_des_task.linear.x = g/he * (com[0]-p_des[0])-dAv[0]
                    com_des_task.linear.y =  g/he * (com[1]-p_des[1])-dAv[1]
                    com_des_task.linear.z =  0
                    com_des_pub.publish(com_des_task)        


                posture_des_task = Accel()
                posture_des_task.linear.x = kp*(q_des[0] - q[22]) - kd*v[21] 
                posture_des_task.linear.y = kp*(q_des[1] - q[23]) - kd*v[22] 
                posture_des_task.linear.z = kp*(q_des[2] - q[24]) - kd*v[23] # 
                posture_des_pub.publish(posture_des_task)    

                rate.sleep()
            ###########################################################################
            ###########################################################################
            ###########################################################################

    ###################################################################################
    ############################ PUBLICA COORDENADAS FINALES ##########################
    ###################################################################################
    while not rospy.is_shutdown():

        if((contador%10==1) and (joint=="Right")):
            joint="Left"
            g=10
            
        elif ((contador%10==1) and (joint=="Left")):
            joint="Right"
            g=9

        p_pos=Point()
        p_vel=Twist()
        p_acc=Accel()
        
        print(g-1)

        p_pos.x=x[g-1][h]
        p_pos.y=y[g-1][h]
        p_pos.z=z[g-1][h]
        
        p_vel.linear.x=0 #x_vel[g][h]
        p_vel.linear.y=0 #y_vel[g][h]
        p_vel.linear.z=0 #z_vel[g][h]
        p_vel.angular.x=0
        p_vel.angular.y=0
        p_vel.angular.z=0

        p_acc.linear.x=0 #x_acc[g][h]
        p_acc.linear.y=0 #y_acc[g][h]
        p_acc.linear.z=0 #z_acc[g][h]
        p_acc.angular.x=0
        p_acc.angular.y=0
        p_acc.angular.z=0

        ########################################################################################################
        ###################ESTO GENERA VARIABLES SOLAMENTE USADAS PARA MULTIREFERENCE###########################
        ########################################################################################################
        
        global x_posicion, y_posicion, z_posicion
        global x_velocidad, y_velocidad, z_velocidad
        global x_aceleracion, y_aceleracion, z_aceleracion

        x_posicion=x[g-1][h]
        y_posicion=y[g-1][h]
        z_posicion=z[g-1][h]

        x_velocidad=0 #x_vel[g][h]
        y_velocidad=0 #y_vel[g][h]
        z_velocidad=0 #z_vel[g][h]

        x_aceleracion=0 #x_acc[g][h]
        y_aceleracion=0 #y_acc[g][h]
        z_aceleracion=0 #z_acc[g][h]
        
        ########################################################################################################
        ########################################################################################################
        ########################################################################################################

        #while(pub_pos.get_num_connections() == 0):
        #    rate.sleep()
        
        pub_pos.publish(p_pos)
        pub_vel.publish(p_vel)
        pub_acc.publish(p_acc)

        equis="Coordenada x: %f" %p_pos.x
        ye   ="Coordenada y: %f" %p_pos.y
        zeta ="Coordenada z: %f \n" %p_pos.z

        equis_vel="Velocidad x: %f" %p_vel.linear.x
        ye_vel   ="Velocidad y: %f" %p_vel.linear.y
        zeta_vel ="Velocidad z: %f \n" %p_vel.linear.z

        equis_acc="Aceleracion x: %f" %p_acc.linear.x
        ye_acc   ="Aceleracion y: %f" %p_acc.linear.y
        zeta_acc ="Aceleracion z: %f \n" %p_acc.linear.z

        print(contador)

        print(joint)
        pub_joint.publish(joint)
        '''
        rospy.loginfo(equis)
        rospy.loginfo(ye)
        rospy.loginfo(zeta)

        rospy.loginfo(equis_vel)
        rospy.loginfo(ye_vel)
        rospy.loginfo(zeta_vel)

        rospy.loginfo(equis_acc)
        rospy.loginfo(ye_acc)
        rospy.loginfo(zeta_acc)
        '''

        contador=contador+1

        rate.sleep()
    ###################################################################################
    ###################################################################################
    ###################################################################################
      
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

#######################################################################################
#######################################################################################
#######################################################################################


