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
        self.alpha_anterior = 0
        self.tetha = 0.0
        self.s = 0.0
    
    def control_law(self, position, velocity, clock_sim):
        # Set Point position for controller
        if self.flag1 == True:
            #print("Inside if flag1")
            self.counter = position
            set_point = self.counter
            self.flag1 = False
        else:
            #print("Inside else flag1")
            set_point = self.counter
        
        """ time_now = self.counter
        time_future_1 = self.counter + 15e-3
        time_future_2 = self.counter + 15e-3

        if self.counter > 70:
            self.counter = 70 """
        time_now = clock_sim
        time_future_1 = time_now + 0.01
        time_future_2 = time_future_1 + 0.01
        tetha_start = 0.0
        tetha_end = 70.0
        time_end = 30

        # print("Position from rhonn_position:{}".format(rhonn_position))
        # print("Position from neuron_1 in time k:{}".format(self.neuron1.fx_0_now))
        # print("Position from neuron_1 in time k+1:{}".format(self.neuron1.fx_0_future))
        # print("Position from sensor:{}".format(position))
        print('Time of programm:{}'.format(time_now))

        # Trajectory Planning
        tetha_now = tetha_start + (((3*pow(time_now,2))/pow(time_end,2))-((2*pow(time_now,3))/pow(time_end,3)))*(tetha_end - tetha_start)
        tetha_future_1 = tetha_start + (((3*pow(time_future_1,2))/pow(time_end,2))-((2*pow(time_future_1,3))/pow(time_end,3)))*(tetha_end - tetha_start)
        tetha_future_2 = tetha_start + (((3*pow(time_future_2,2))/pow(time_end,2))-((2*pow(time_future_2,3))/pow(time_end,3)))*(tetha_end - tetha_start)
        
        if time_now > time_end:
            tetha_now = tetha_end
            tetha_future_1 = tetha_end
            tetha_future_2 = tetha_end

        self.tetha = tetha_now        
        

        # Input - Output Linealization
        k1 = 0.09 #0.75 0.09
        k2 = 0.03 #0.01 0.03
        k3 = 0.08 #     0.05
        self.error_x0 = self.neuron1.fx_0_now - tetha_now
        self.error_x1 = self.neuron1.fx_0_future - tetha_future_1

        # Linear solution
        # v = -(k1*self.error_x0) - (k2*self.error_x1)
        
        # Super Twisting
        self.s = (k2*self.error_x0) + self.error_x1
        #s = (k2*self.neuron1.fx_0_now) + self.neuron2.fx_1_now
        
        if self.s < 0:
            sign_s = -1
        else:
            sign_s = 1
        
        alpha_siguiente = self.alpha_anterior + (0.01*(-k3 * sign_s))
        v = -k1*np.sqrt(np.abs(self.s))*sign_s + self.alpha_anterior
        self.alpha_anterior = alpha_siguiente

        #v = (-k3 * sign_s)

        """ old_control_law = (1.0/(self.neuron1.W1_fixed*self.neuron2.W2_fixed))*(v - (self.neuron1.w_weight[0,0]*activation_function(self.neuron1.fx_0_future)) - self.neuron1.w_weight[1,0]
                                                                - (self.neuron1.W1_fixed *((self.neuron2.w_weight[0,0]*activation_function(position))
                                                                                           + (self.neuron2.w_weight[1,0]*activation_function(velocity)))) 
                                                                                           + tetha_future_2 ) """
        control_law = -(self.neuron1.w_weight[1,0] - v - tetha_future_2 + self.neuron1.W1_fixed * ((self.neuron2.w_weight[0,0]*activation_function(position))+(self.neuron2.w_weight[1,0]*activation_function(velocity))) + (self.neuron1.w_weight[0,0]*activation_function(self.neuron1.fx_0_future)))/(self.neuron1.W1_fixed*self.neuron2.W2_fixed)
        # control_law = 13.8
        
        # Block Control
        # k1 = 5e-3
        # k2 = 1e-4
        # self.error_x0 = self.neuron1.fx_0_now - tetha_now
        # self.error_x1 = self.neuron2.fx_1_now - 0
        # self.error_x0_1 = self.neuron1.fx_0_future - tetha_future_1

        # control_law = (1.0/self.neuron2.W2_fixed)*((k2*self.error_x1)-(self.neuron2.w_weight[0,0]*activation_function(position))
        #                                  -(self.neuron2.w_weight[1,0]*activation_function(velocity))
        #                                  +((1.0/self.neuron1.W1_fixed)*(-(self.neuron1.w_weight[0,0]*activation_function(self.neuron1.fx_0_future))-self.neuron1.w_weight[1,0]+tetha_future_2+(k1*self.error_x0_1))))
        
        #Inverse Optimal Control
        #f = [(self.neuron1.w_weight[0,0]*activation_function(rhonn_position)) + self.neuron1.w_weight[1,0] + (self.neuron1.W1_fixed * velocity); ]

        # Open loop (spline)
        # angulo_temporal = (self.counter*np.pi)/180
        # if angulo_temporal > 360:
        #     self.counter = 0

        # ueq = 10*np.sin(angulo_temporal*1) #Valor maximo de 10 a 14
        # self.counter = self.counter + 1


        if np.abs(control_law) <= 20:
           self.u = control_law
        else:
           self.u = (20*(control_law/np.abs(control_law)))

        return self.u