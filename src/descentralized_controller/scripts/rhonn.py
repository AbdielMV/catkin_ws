#!/usr/bin/env python
import numpy as np
from controller import Controller

def activation_function(state):
    return np.tanh(state)

class Rhonn:
    def __init__(self, c_input, w_input, number):

        self.z_input = c_input
        self.w_weight = w_input
        self.neuron = number
        self.W1_fixed = 0.001 #0.001
        self.W2_fixed = 1 #0.1
        self.u = 0.0
        # self.ueq = None
        # self.v = None
        # self.fx_0 = None
        # self.fx_1 = None
        # self.error_x0 = None
        # self.error_x1 = None
        # self.error_x1_old = None
        # self.counter = None
        # self.sign_ex0 = None
        # self.sign_ex1 = None
        self.observer_x0_actual = 0.0
        self.observer_x1_actual = 0.0
        self.observer_x0_prediction = 0.0
        self.observer_x1_prediction = 0.0
        # self.error_obs1 = None
        # self.error_obs2 = None
        # self.sign_eobs1 = None
        # self.sign_eobs2 = None
        # self.v1 = None
        # self.v2 = None
        # self.fixed_result = None
        # self.state_final_prediction = None
        # self.fx_11 = None
        # self.fx_12 = None
        # self.fx_21 = None
        # self.fx_22 = None
        # self.fx_23 = None
        # self.set_point = None

    def observer_state(self, position, velocity):
        k1 = 0.01
        k2 = 0.001
        self.observer_x0_actual = self.observer_x0_prediction
        self.observer_x1_actual = self.observer_x1_prediction
        error_obs1 = position - self.observer_x0_actual
        error_obs2 = velocity - self.observer_x1_actual
        if error_obs1 < 0:
            sign_eobs1 = -1
        else:
            sign_eobs1 = 1
        v1 = -k1*np.sqrt(abs(error_obs1))*sign_eobs1
        v2 = -k2*sign_eobs1
        self.observer_x0_prediction = position + v1
        self.observer_x1_prediction = velocity + v2
        if self.neuron == 1:
            return self.observer_x0_prediction
        else:
            return self.observer_x1_prediction

    def prediction_state(self, position, velocity):
        # self.update_input(position,velocity)
        w_transposed = self.w_weight.transpose()
        if self.neuron == 1:    
            # fx_11 = w_transposed[0,0]*self.z_input[0,0]
            # fx_12 = w_transposed[0,1]*self.z_input[1,0]
            fixed_result = self.W1_fixed*velocity
            # state_final_prediction = fx_11+fx_12+fixed_result
            state_final_prediction = (w_transposed[0,0]*np.tanh(position)) + w_transposed[0,1] + fixed_result
            fx_0 = state_final_prediction
            return fx_0
        else:
            # fx_21 = w_transposed[0,0]*self.z_input[0,0]
            # fx_22 = w_transposed[0,1]*self.z_input[1,0]
            # fx_23 = w_transposed[0,2]*self.z_input[2,0]
            fixed_result = self.W2_fixed*self.u
            #state_final_prediction = fx_21+fx_22+fx_23+fixed_result
            state_final_prediction = (w_transposed[0,0]*np.tanh(position)) + (w_transposed[0,1]*np.tanh(velocity)) + w_transposed[0,2] + fixed_result
            fx_1 = state_final_prediction
            return fx_1
    

    def update_input(self, position, velocity):
        if self.neuron == 1:
            self.z_input[0,0] = activation_function(position)
        else:
            self.z_input[0,0] = activation_function(position)
            self.z_input[1,0] = activation_function(velocity)
        pass

    def update_weights(self, new_weights):
        self.w_weight = new_weights

    def get_inputs(self):
        return self.z_input

    def get_neuron(self):
        return self.neuron

    def get_weights(self):
        return self.w_weight

    def get_control_law(self, controller_object):
        self.u = controller_object.u

    def fx0_value(self, rhonn_object):
        self.fx_12 = rhonn_object.fx_12
