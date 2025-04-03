# -*- coding: utf-8 -*-
"""
Created on Wed Apr  2 20:56:01 2025

@author: siddh
"""

import numpy as np

class flight_params:
    # Rotation Matrix that represents a fixed rotation of PI/2 rad around the X-Axis
    RotMat_X_PI_2_list = [[1,  0,  0],
                          [0,  0,  1],
                          [0, -1,  0]]
    
    # RotMat_X_PI_2 = chrono.ChMatrix33D()
    # RotMat_X_PI_2.SetMatr(RotMat_X_PI_2_list)
    RotMat_X_PI_2_array = np.array(RotMat_X_PI_2_list)
    
    
    
    # Rotation Matrix that represents a fixed rotation of -PI/2 rad around the X-Axis
    RotMat_X_PI_2_tran_array = np.transpose(RotMat_X_PI_2_array)
    
    # Inertia matrix of the system: (drone frame + box + propellers) espressed in Solidworks coordinate sys (x-front, y-up, z-right), computed at the center of mass
    Inertia_mat_sw = np.array([[ 0.0227199027,  0.0010034978, -0.0000065570],
                               [ 0.0010034978,  0.0161469360, -0.0000056584],
                               [-0.0000065570, -0.0000056584,  0.0220204718]])
    
    # Inertia matrix of the system: (drone frame + box + propellers) espressed in Pixhawk coordinate sys (x-front, y-right, z-down), computed at the center of mass
    Inertia_mat_pixhawk = np.matmul(RotMat_X_PI_2_array, np.matmul(Inertia_mat_sw, RotMat_X_PI_2_tran_array))
    
    surface_area = 0.07 # Surface area of the drone to account for drag [m^2]
    air_density = 1.225 # Air density [kg/m^3]
    #drag_coefficient = 1.28 # Drag coefficient (equal to that of a plate) [-]
    drag_coefficient = 1.28 # Drag coefficient (equal to that of a plate) [-]
    drag_coefficient_matrix = np.matrix(np.diag([drag_coefficient,drag_coefficient,0]))
    
    def __init__(self):
        self.G_acc = 9.80665 # value of g
        self.mass_total_estimated = 2.025 # [kg] 2.025
        self.I_matrix_estimated = np.matrix(self.Inertia_mat_pixhawk)
        self.air_density_estimated = self.air_density
        self.surface_area_estimated = self.surface_area
        self.drag_coefficient_estimated = self.drag_coefficient
        self.drag_coefficient_matrix_estimated = np.matrix(np.diag([self.drag_coefficient_estimated,self.drag_coefficient_estimated,0]))