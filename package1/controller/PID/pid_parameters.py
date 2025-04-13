# -*- coding: utf-8 -*-
"""
Created on Sun Feb 18 21:49:06 2024

@author: grem6
"""

import numpy as np
import math
  
class PID_parameters:
  def __init__(self):
    # self.number_of_states = 0
    # self.size_DATA = 0
    # self.KP_tran = np.matrix(np.zeros((3,3)))
    # self.KD_tran = np.matrix(np.zeros((3,3)))
    # self.KI_tran = np.matrix(np.zeros((3,3)))
    # self.KP_rot = np.matrix(np.zeros((3,3)))
    # self.KD_rot = np.matrix(np.zeros((3,3)))
    # self.KI_rot = np.matrix(np.zeros((3,3)))
    # self.sphereEpsilon = 0
    # self.maximumThrust = 0
    # self.EllipticConeEpsilon = 0
    # self.maximumRollAngle = 0
    # self.maximumPitchAngle = 0
    # self.planeEpsilon = 0
    # self.alphaPlane = 0
    
    # Number of states to be integrated by RK4
    self.number_of_states = 10
    # Length of the array vector that will be exported 
    self.size_DATA = 103 
    # ----------------------------------------------------------------
    #                   Safety Mechanism Parameters
    # ----------------------------------------------------------------

    # Mu - sphere intersection
    self.sphereEpsilon = 1e-2
    self.maximumThrust = 85 # [N] 85

    # Mu - elliptic cone intersection
    self.EllipticConeEpsilon = 1e-2
    self.maximumRollAngle = math.radians(32) # [rad] 25
    self.maximumPitchAngle = math.radians(32) # [rad] 25

    # Mu - plane intersection
    self.planeEpsilon = 1e-2
    self.alphaPlane = 0.95 # [-] coefficient for setting the 'height' of the bottom plane. Must be >0 and <1.
    

