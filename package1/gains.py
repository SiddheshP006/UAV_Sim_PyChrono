# -*- coding: utf-8 -*-
"""
Created on Thu Mar 20 02:02:02 2025

@author: siddh
"""

import numpy as np
import math

class Gains:
    
    def __init__(self):
            
        # Roll filters gains (from matlab)
        self.A_phi_ref = np.matrix([[-15, -225],[1, 0]])
        self.B_phi_ref = np.array([[1],[0]])
        self.C_phi_ref = np.matrix([225, 0])
        self.D_phi_ref = 0
            
        # Pitch filters gains (from matlab)
        self.A_theta_ref = np.matrix([[-15, -225],[1, 0]])
        self.B_theta_ref = np.array([[1],[0]])
        self.C_theta_ref = np.matrix([225, 0])
        self.D_theta_ref = 0
        
        self.KP_tran = np.matrix(1 * np.diag([5,5,6]))
        self.KD_tran = np.matrix(1 * np.diag([8,8,3]))
        self.KI_tran = np.matrix(1 * np.diag([1,1,1]))

        # **Rotational** PID parameters
        self.KP_rot = np.matrix(1 * np.diag([100,100,50]))
        self.KD_rot = np.matrix(1 * np.diag([50,50,50]))
        self.KI_rot = np.matrix(1 * np.diag([20,20,10]))