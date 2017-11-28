# -*- coding: utf-8 -*-
"""
Created on Tue Nov 28 09:50:52 2017

@author: Luc
"""

import numpy as np
import scipy.linalg as spl
from math import *

std_a = 1.
std_yawdd = 1.

xk = np.array([[.312243],[.58034],[2.80],[0],[0]], dtype = np.float64)

Pk = np.eye(5, dtype = np.float64)

for timestep in range(0,2500,5):
    print(timestep/100)
    dt = 0.05
    xa = np.zeros((7, 1), dtype = np.float64)
    xa[:5] = xk
    
    Pa = np.ndarray((7,7), dtype = np.float64)
    Pa[:5, :5] = Pk  
    Pa[5,5] = std_a
    Pa[6,6] = std_yawdd
      
    A = spl.sqrtm(Pa)
    
    if not np.array_equal(A.dot(A) , Pa):
        print("Error in square root matrix")
        print(A.dot(A))
        print(Pa)
        break
    
    Xak = np.concatenate([xa, xa + np.sqrt(3) * A, xa - np.sqrt(3) * A], axis = 1)
    
    Xk1 = np.ndarray((5, 15), np.float64)
    
    for i in range(0, 15):
        px = Xak[0, i]
        py = Xak[1, i]
        v = Xak[2, i]
        psi = Xak[3, i]
        psi_d = Xak[4, i]
        nu_a = Xak[5, i]
        nu_pdd = Xak[6, i]
        
        sin_psi = sin(psi)
        cos_psi = cos(psi)
        sin_l = sin(psi + psi_d * dt) - sin_psi
        cos_l = cos_psi - cos(psi + psi_d * dt)               
        
        if abs(psi_d) >= 0.0001:
            Xk1[0, i] = Xak[0, i] + (v / psi_d) * sin_l + 0.5 * dt**2 * cos_psi * nu_a
            Xk1[1, i] = Xak[1, i] + (v / psi_d) * cos_l + 0.5 * dt**2 * sin_psi * nu_a
        else:
            Xk1[0, i] = Xak[0, i] + v * cos_psi * dt + 0.5 * dt**2 * cos_psi * nu_a
            Xk1[1, i] = Xak[1, i] + v * sin_psi * dt + 0.5 * dt**2 * sin_psi * nu_a
        
        Xk1[2, i] = Xak[2, i] + dt * nu_a
        Xk1[3, i] = Xak[3, i] + psi_d * dt + 0.5 * dt**2 * nu_pdd
        Xk1[4, i] = Xak[4, i] + dt * nu_pdd
            
    weights = np.array([[-1.33333],[.166667],[.166667],[.166667],[.166667],[.166667],[.166667],[.166667],[.166667],[.166667],[.166667],[.166667],[.166667],[.166667],[.166667]],
                       dtype = np.float64)
    
    xk = np.dot(Xk1, weights)
    Pk = np.zeros_like(Pk)    
    
    for i in range(0, 15):
        Pk += weights[i] * np.matmul((Xk1[:, i] - xk.transpose()).transpose(), (Xk1[:, i] - xk.transpose()))
        
