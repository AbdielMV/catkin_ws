#!/usr/bin/env python

import rospy
import numpy as np
import time
import csv
import matplotlib.pyplot as plt
from std_msgs.msg import String
from whole_body_state_msgs.msg import JointState, WholeBodyTrajectory, WholeBodyState, RhonnState
from rhonn import Rhonn
from efk import Efk
from controller import Controller

# Initialize RHONN values (inputs and weights)
C1 = np.array([1, 1], dtype=float).reshape((2, 1))
C2 = np.array([1, 1], dtype=float).reshape((2, 1))

W1 = np.array([0, 0], dtype=float).reshape((2, 1))
W2 = np.array([0, 0], dtype=float).reshape((2, 1))

# Initialize values of EFK training (P, Q, and R)
R1 = 1e5 * np.identity(1, dtype=float)
Q1 = 1e5 * np.identity(2, dtype=float)
P1 = 1e10 * np.identity(2, dtype=float)

R2 = 1e6 * np.identity(1, dtype=float)
Q2 = 1e8 * np.identity(2, dtype=float)
P2 = 1e15 * np.identity(2, dtype=float)


# Create object Rhonn
neuron_1 = Rhonn(C1, W1, 1)
neuron_2 = Rhonn(C2, W2, 2)

# Create object EFK
efk_object_1 = Efk(R1,P1,Q1)
efk_object_2 = Efk(R2,P2,Q2)

#Create object Controller
controller_object = Controller(neuron_1,neuron_2)

# Create variables for message
position_msg = WholeBodyState()
joint_estimation = JointState()
rhonn_estimation = RhonnState()

# Create simple variables
position_1 = 0.0
velocity_1 = 0.0
position_y1 = 0.0
velocity_y1 = 0.0

# Initialize global variables
position = 0.0
velocity = 0.0
name = ""

# Internal Clock Simulation
clock_sim = 0.0
dt = 0.00001
dt_controller = 0.01
time_end = 15.0

def sensor_callback(data):
    global position_msg, joint_estimation, rhonn_estimation, name, position, velocity, position_1, velocity_1, position_y1, velocity_y1

    name = data.joints[21].name
    position = data.joints[21].position
    velocity = data.joints[21].velocity

    # Convert from radian to degrees
    position = (position*180)/np.pi
    velocity = (velocity*180)/np.pi

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

def processing_data(event):
    global position_msg, joint_estimation, rhonn_estimation, name, position, velocity, position_1, velocity_1, position_y1, velocity_y1, clock_sim
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
    neuron_2.get_control_law(controller_object) #Its just to get u, not calculate u

    

    # Construction of Message
    rhonn_estimation.name = name
    rhonn_estimation.position = neuron_1_value
    rhonn_estimation.velocity = neuron_2_value
    rhonn_estimation.error_w1 = error_1
    rhonn_estimation.error_w2 = error_2
    rhonn_estimation.obs_position = observer_1_value
    rhonn_estimation.obs_velocity = observer_2_value
    rhonn_estimation.reference = controller_object.tetha

    joint_estimation.name = name
    joint_estimation.position = position
    joint_estimation.velocity = velocity
    joint_estimation.effort = neuron_2.u

def control_law_calculation(event):
    controller_object.control_law(position,velocity, clock_sim) #Here is calculated the control law

def save_to_csv(data):
        with open('vector_data.csv', 'wb') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Time', 'position', 'velocity', 'rhonn_x0', 'rhonn_x1', 'error_x0', 'error_x1', 'set_point', 'effort', 'error_tracking_x0', 'error_tracking_x1', 'surface', 'w11', 'w12', 'w21', 'w22'])  # Assuming 3D vectors
            writer.writerows(data)
        rospy.loginfo('Data saved to vector_data.csv')

def ending_node(pub):
    rospy.loginfo('Ending Program, home position and effort cero')
    global position_msg, joint_estimation, rhonn_estimation, name, position, velocity
    joint_estimation.name = name
    joint_estimation.position = position
    joint_estimation.velocity = velocity
    joint_estimation.effort = 0.5
    position_msg.joints.append(joint_estimation)
    pub.publish(position_msg)

def talker():
    global position_msg, joint_estimation, rhonn_estimation, name, position, velocity, clock_sim
    rospy.init_node('identifier', anonymous=True)
    pub = rospy.Publisher('/reemc/efforts', WholeBodyState, queue_size=1)
    rate = rospy.Rate(1e2) # 10hz
    rospy.Subscriber('/robot_states', WholeBodyState, sensor_callback, queue_size=1)
    rospy.Timer(rospy.Duration(dt), processing_data)
    rospy.Timer(rospy.Duration(dt_controller), control_law_calculation)
    data = []

    #while not rospy.is_shutdown():
    while clock_sim < time_end:   #and left_time > 0

        # Clear the messages to avoid accumulation
        position_msg.joints = []
        position_msg.rhonn = []

        # Has to be JointState (joints) NOT Rhonn (rhonn)
        position_msg.joints.append(joint_estimation)
        position_msg.rhonn.append(rhonn_estimation)

        # Append position and velocity to data list
        data.append([rospy.get_time(), joint_estimation.position, joint_estimation.velocity, rhonn_estimation.position, rhonn_estimation.velocity, rhonn_estimation.error_w1, rhonn_estimation.error_w2, controller_object.tetha, joint_estimation.effort, controller_object.error_x0, controller_object.error_x1, controller_object.s, neuron_1.w_weight[0,0], neuron_1.w_weight[1,0], neuron_2.w_weight[0,0], neuron_2.w_weight[1,0]])

        # Publish the message
        position_msg.header.stamp = rospy.Time.now()
        position_msg.time = rospy.get_time()
        pub.publish(position_msg)
        rate.sleep()
        clock_sim = clock_sim + dt_controller

    save_to_csv(data)
    print("Final Time clock_sim:{}".format(clock_sim))
    ending_node(pub)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        #ending_node()
        pass
