#!/usr/bin/env python
import numpy as np

class Controller:
    def __init__(self, neuron_1, neuron_2):
        self.neuron1 = neuron_1
        self.neuron2 = neuron_2
        self.error_x1 = 0.0
    
    def control_law(self, position, velocity):
        set_point = 45
        error_x1_old = self.error_x1
        self.error_x0 = position - set_point
        self.error_x1 = velocity - 0

        if self.error_x0 < 0:
            sign_ex0 = -1
        else:
            sign_ex0 = 1
        if self.error_x1 < 0:
            sign_ex1 = -1
        else:
            sign_ex1 = 1
        k1 = 0.65
        k2 = 1e-8
        v = -(k1*self.error_x0) - (k2*self.error_x1)
        ueq = v
        if np.abs(ueq) <= 15:
            u = ueq
        else:
            u = (15*(ueq/np.abs(ueq)))
        return u
        