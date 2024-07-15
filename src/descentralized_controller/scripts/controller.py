#!/usr/bin/env python
import numpy as np
from scipy.interpolate import interpolate

def activation_function(state):
    return np.tanh(state)

class Controller:
    def __init__(self, neuron_1, neuron_2):
        self.neuron1 = neuron_1
        self.neuron2 = neuron_2
        self.error_x0 = 0.0
        self.error_x1 = 0.0
        self.error_x0_1 = 0.0
        self.u = 0.0
        self.ueq = 0.0
        self.counter = 0
        self.flag1 = True
    
    def control_law(self, position, velocity, rhonn_position, rhonn_velocity):
        # Set Point position for controller
        if self.flag1 == True:
            self.counter = position
            self.flag1 = False
        
        time_now = self.counter
        time_future_1 = self.counter + 0.015
        time_future_2 = self.counter + 0.030
        if self.counter > 70:
            self.counter = 70
        self.counter = self.counter + 0.015
        set_point = self.counter
        #print("set point: {}".format(set_point))

        if self.error_x0 < 0:
            sign_ex0 = -1
        else:
            sign_ex0 = 1
        if self.error_x1 < 0:
            sign_ex1 = -1
        else:
            sign_ex1 = 1

        # Input - Output Linealization
        k1 = 0.00000000000075 #0.75
        k2 = 0.0000000000000001 #0.01
        v = -(k1*self.error_x0) - (k2*self.error_x1)
        self.error_x0 = position - set_point
        self.error_x1 = rhonn_position - time_future_1

        ueq = (1.0/(self.neuron1.W1_fixed*self.neuron2.W2_fixed))*(v - (self.neuron1.w_weight[0,0]*activation_function(rhonn_position)) - self.neuron1.w_weight[1,0]
                                                                - (self.neuron1.W1_fixed *((self.neuron2.w_weight[0,0]*activation_function(position))
                                                                                           + (self.neuron2.w_weight[1,0]*activation_function(velocity)))) 
                                                                                           + time_future_2 )
        
        # Block Control
        # k1 = -3.5
        # k2 = -1e-3
        # self.error_x0 = position - set_point
        # #print("error_x0: {}".format(self.error_x0))
        # self.error_x1 = velocity - 0
        # #print("error_x1: {}".format(self.error_x1))
        # self.error_x0_1 = rhonn_position - time_future_1
        # # print("error_x0_1: {}".format(self.error_x0_1))
        
        # # print("1/self.neuron2.W2_fixed: {}".format(1.0/self.neuron2.W2_fixed))
        # # print("k2*self.error_x1: {}".format(k2*self.error_x1))

        # ueq = (1.0/self.neuron2.W2_fixed)*((k2*self.error_x1)-(self.neuron2.w_weight[0,0]*activation_function(position))
        #                                  -(self.neuron2.w_weight[1,0]*activation_function(velocity))
        #                                  +((1.0/self.neuron1.W1_fixed)*(-(self.neuron1.w_weight[0,0]*activation_function(rhonn_position))-self.neuron1.w_weight[1,0]+time_future_2+(k1*self.error_x0_1))))
        
        # Open loop (spline)
        # angulo_temporal = (self.counter*np.pi)/180
        # if angulo_temporal > 360:
        #     self.counter = 0

        # ueq = 10*np.sin(angulo_temporal*1) #Valor maximo de 10 a 14
        # self.counter = self.counter + 1


        if np.abs(ueq) <= 15:
           self.u = ueq
        else:
           self.u = (15*(ueq/np.abs(ueq)))

        return self.u