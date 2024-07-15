#!/usr/bin/env python
import rospy
import numpy as np
from rhonn import Rhonn

class Efk:
    def __init__(self, R, P, Q):
        self.p_now = P 
        self.p_next = P
        self.q_now = Q
        self.q_next = Q
        self.w_now = np.array([0, 0], dtype=float).reshape((2, 1))
        self.w_next = np.array([0, 0], dtype=float).reshape((2, 1))
        self.error_1 = 0
        self.error_2 = 0
        self.r = R

    def error_estimation(self, state, position, velocity, neuron_id):
        if neuron_id == 1:
            self.error_1 = position - state
            error_return = self.error_1
        else:
            self.error_2 = velocity - state
            error_return = self.error_2
        return error_return

    def calculate_new_weights(self, rhonn_object):
        etha = 0.7
        h = rhonn_object.get_inputs()
        neuron = rhonn_object.neuron
        self.w_now = rhonn_object.w_weight
        m = 1/((self.r + h.transpose().dot(self.p_now).dot(h)))
        k = self.p_now.dot(h).dot(m)
        if neuron==1:
            self.w_next = self.w_now + (etha * k * self.error_1)
        else:
            self.w_next = self.w_now + (etha * k * self.error_2)
        self.p_next = self.p_now - (k.dot(h.transpose()).dot(self.p_now)) + self.q_now
        self.update_values()
        rhonn_object.update_weights(self.w_now)
    
    def update_values(self):
        self.p_now = self.p_next
        self.w_now = self.w_next