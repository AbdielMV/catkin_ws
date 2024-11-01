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
        self.W1_fixed = 10 #0.001 100  10
        self.W2_fixed = 10 #0.1   10   10
        self.u = 0.0
        self.observer_x0_actual = 0.0
        self.observer_x1_actual = 0.0
        self.observer_x0_prediction = 0.0
        self.observer_x1_prediction = 0.0
        self.fx_0_now = 0.0
        self.fx_0_future = 0.0
        self.fx_1_now = 0.0
        self.fx_1_future = 0.0

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
        w_transposed = self.w_weight.transpose()
        if self.neuron == 1:
            self.fx_0_now = self.fx_0_future    
            self.z_input[0,0] = activation_function(position)
            fixed_result = self.W1_fixed*velocity
            state_final_prediction = (w_transposed[0,0]*self.z_input[0,0]) + (w_transposed[0,1]*self.z_input[1,0]) + fixed_result
            self.fx_0_future = state_final_prediction
            return self.fx_0_future
        else:
            self.fx_1_now = self.fx_1_future
            self.z_input[0,0] = activation_function(position)
            self.z_input[1,0] = activation_function(velocity)
            fixed_result = self.W2_fixed*self.u
            state_final_prediction = (w_transposed[0,0]*self.z_input[0,0]) + (w_transposed[0,1]*self.z_input[1,0]) + fixed_result
            self.fx_1_future = state_final_prediction
            return self.fx_1_future

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
