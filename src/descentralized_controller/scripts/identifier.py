#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String
from whole_body_state_msgs.msg import JointState, WholeBodyTrajectory, WholeBodyState, RhonnState
from rhonn import Rhonn
from efk import Efk
from controller import Controller

# Initialize RHONN values (inputs and weights)
C1 = np.array([1, 1], dtype=float).reshape((2, 1))
C2 = np.array([1, 1, 1], dtype=float).reshape((3, 1))

W1 = np.array([0, 0], dtype=float).reshape((2, 1))
W2 = np.array([0, 0, 0], dtype=float).reshape((3, 1))

# Initialize values of EFK training (P, Q, and R)
R1 = 1e5 * np.identity(1, dtype=float)
Q1 = 1e5 * np.identity(2, dtype=float)
P1 = 1e10 * np.identity(2, dtype=float)

R2 = 1e3 * np.identity(1, dtype=float)
Q2 = 1e3 * np.identity(3, dtype=float)
P2 = 1e10 * np.identity(3, dtype=float)

# print ("C1: {}".format(C1))
# print ("C2 {}".format(C2))
# print ("W1: {}".format(W1))
# print ("W2: {}".format(W2))

# Create object Rhonn
neuron_1 = Rhonn(C1, W1, 1)
neuron_2 = Rhonn(C2, W2, 2)

# Create object EFK
efk_object_1 = Efk(R1,P1,Q1)
efk_object_2 = Efk(R2,P2,Q2)

#Create object Controller
control_law = Controller(neuron_1,neuron_2)

# Create variables for message
position_msg = WholeBodyState()
joint_estimation = JointState()
rhonn_estimation = RhonnState()

# Create simple variables
position_1 = 0.0
velocity_1 = 0.0
position_y1 = 0.0
velocity_y1 = 0.0 

def callback(data):
    global position_msg, joint_estimation, rhonn_estimation, position_1, velocity_1, position_y1, velocity_y1
    print("Inside callback")
    name = data.joints[21].name
    position = (data.joints[21].position*180)/np.pi
    velocity = (data.joints[21].velocity*180)/np.pi

    # Filter first degree
    position_y = position_y1*0.99+position_1*0.00995
    velocity_y = velocity_y1*0.99+velocity_1*0.00995
    position_1 = position
    velocity_1 = velocity
    position_y1 = position_y
    velocity_y1 = velocity_y

    # Change of variable for use those names
    position = position_y
    velocity = velocity_y
    # print ("name: {}".format(name))
    # print ("position {}".format(position))
    # print ("velocity: {}".format(velocity))

    #print("Neurona 1")
    observer_1_value = neuron_1.observer_state(position, velocity)
    neuron_1_value = neuron_1.prediction_state(position,velocity)
    error_1 = efk_object_1.error_estimation(neuron_1_value,position,velocity,1)
    efk_object_1.calculate_new_weights(neuron_1)

    #print("Neuron 2")
    observer_2_value = neuron_2.observer_state(position, velocity)
    neuron_2_value = neuron_2.prediction_state(position,velocity)
    error_2 = efk_object_2.error_estimation(neuron_2_value,position,velocity,2)
    efk_object_2.calculate_new_weights(neuron_2)

    # Construction of Message
    rhonn_estimation.name = name
    rhonn_estimation.position = neuron_1_value
    rhonn_estimation.velocity = neuron_2_value
    rhonn_estimation.error_w1 = error_1
    rhonn_estimation.error_w2 = error_2
    rhonn_estimation.obs_position = observer_1_value
    rhonn_estimation.obs_velocity = observer_2_value

    # Debug messages (optional)
    print("Building message")
    # print("Valor X1_k+1 =", neuron_1_value)
    # print("Valor X2_k+1 =", neuron_2_value)

    joint_estimation.name = name
    joint_estimation.position = position
    joint_estimation.velocity = velocity
    #joint_estimation.effort = neuron_2.getControlLaw()
    print ("End callback")

def talker():
    global position_msg, joint_estimation, rhonn_estimation
    rospy.init_node('identifier', anonymous=True)
    pub = rospy.Publisher('/reemc/efforts', WholeBodyState, queue_size=10)
    rospy.Subscriber('/robot_states', WholeBodyState, callback, queue_size=10)
    rate = rospy.Rate(100) # 10hz

    while not rospy.is_shutdown():
        # Clear the messages to avoid accumulation
        position_msg.joints = []
        position_msg.rhonn = []

        #Has to be JointState (joints) NOT Rhonn (rhonn)
        position_msg.joints.append(joint_estimation)
        position_msg.rhonn.append(rhonn_estimation)

        # Publish the message
        position_msg.header.stamp = rospy.Time.now()
        position_msg.time = rospy.get_time()
        print("Before publishing")
        pub.publish(position_msg)
        print("After publishing")
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
