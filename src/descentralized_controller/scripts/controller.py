#!/usr/bin/env python
import numpy as np
from scipy.interpolate import interpolate

class Controller:
    def __init__(self, neuron_1, neuron_2):
        self.neuron1 = neuron_1
        self.neuron2 = neuron_2
        self.error_x1 = 0.0
        self.u = 0.0
        self.counter = 0
        self.flag1 = True
    
    def control_law(self, position, velocity):
        # Set Point position for controller
        # set_point = 90
        if self.flag1 == True:
            self.counter = position
            self.flag1 = False
        
        self.counter = self.counter + 1e-1
        if self.counter > 40:
            self.counter = 40
        set_point = self.counter
        print("set point: {}".format(set_point))

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
        k1 = 0.3
        k2 = 1e-8
        v = -(k1*self.error_x0) - (k2*self.error_x1)
        ueq = v
        
        # Open loop (spline)
        # angulo_temporal = (self.counter*np.pi)/180
        # if angulo_temporal > 360:
        #     self.counter = 0

        # ueq = 10*np.sin(angulo_temporal*1) #Valor maximo de 10 a 14
        #self.counter = self.counter + 1


        if np.abs(ueq) <= 15:
           self.u = ueq
        else:
           self.u = (15*(ueq/np.abs(ueq)))

        return self.u