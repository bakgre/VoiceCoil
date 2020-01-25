# -*- coding: utf-8 -*-
"""
Created on Mon Dec 17 18:47:44 2018

@author: Greg
"""

import numpy as np
import control as ctrl
import matplotlib.pyplot as plt

def showResponses(plantNum, plantDen):
    tf = ctrl.TransferFunction(plantNum, plantDen)
    tfCLden = np.polyadd(np.array(plantNum), np.array(plantDen))
    tfCL = ctrl.TransferFunction(plantNum,tfCLden)
    # Open Loop Plant
    resp = ctrl.step_response(tf, np.linspace(0., 10., 100))
    plt.ylim(np.min(resp[1]), np.max(resp[1]))
    plt.plot(resp[0],resp[1])
    plt.title("Open loop plant")
    plt.show()
    ctrl.root_locus(tf)
    plt.ylim(-10, 10)
    plt.xlim(-10, 10)
    plt.show()
    # Closed Loop Plant
    resp = ctrl.step_response(tfCL, np.linspace(0., 10., 100))
    plt.ylim(np.min(resp[1]), np.max(resp[1]))
    plt.plot(resp[0],resp[1])
    plt.title("Closed loop plant")
    plt.show()
    ctrl.root_locus(tfCL)
    plt.ylim(-10, 10)
    plt.xlim(-10, 10)
    plt.show()

    #Multiply PI Controller * plant response
    PIGain = 6
    PIZero = -6.0
    PInum = np.array([1.0, -PIZero]) # zero at s=-PIZero
    PIden = np.array([1, 0])  # pole at origin
    PITF = ctrl.TransferFunction(np.polymul(PInum,plantNum), np.polymul(PIden,plantDen))
    PITF = PIGain * PITF

    #Compute and show PI controller * plant response
    resp = ctrl.step_response(ctrl.feedback(PITF), np.linspace(0., 10., 100))
    plt.ylim(np.min(resp[1]), np.max(resp[1]))
    plt.plot(resp[0],resp[1])
    plt.title("PI control ofplant")
    plt.show()
    #Draw root locus diagram
    ctrl.root_locus(PITF)
    plt.ylim(-50, 50)
    plt.show()

    #Now Compute and show PID controller * plant response
    PIDGain = 10.0
    PIDZero1 = -1.0
    PIDZero2 = -2.0

    PIDden = np.array([1.0, 0.0])  # pole at origin
    PIDnum = np.array(np.polymul([1.0, -PIDZero1], [1.0, -PIDZero2])) # 2 zeros 
    PIDTF = ctrl.TransferFunction(np.polymul(PIDnum,plantNum), np.polymul(PIDden,plantDen))
    PIDTF = PIDGain * PIDTF

    resp = ctrl.step_response(ctrl.feedback(PIDTF), np.linspace(0., 10., 100))
    plt.ylim(np.min(resp[1]), np.max(resp[1]))
    plt.plot(resp[0],resp[1])
    plt.title("PID control ofplant")
    plt.show()
    #Draw root locus diagram
    ctrl.root_locus(PIDTF)
    plt.ylim(-25, 25)
    plt.xlim(-40, 10)
    plt.show()


#Plant Transfer Function
# Plant poles at s=-2+-2i
num = np.array([1])
den = np.array([1., 4., 8.])
showResponses(num, den)

# SIngle pole at origin s=0
num = np.array([1])
den = np.array([1.,0.])
showResponses(num, den)
#

# Double pole at origin s=0
num = np.array([1])
den = np.array([1.,0., 0.])
showResponses(num, den)
#
