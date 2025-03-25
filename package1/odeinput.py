# -*- coding: utf-8 -*-
"""
Created on Thu Mar 20 03:09:47 2025

@author: siddh
"""

import math
import numpy as np
from package1.functions import *

class ode_inputs:
    
    def __init__(self, euler_angles, yaw_ref, yaw_ref_dot, yaw_ref_ddot, angular_vel, angular_acc, trans_pos_NED, trans_vel_NED, trans_pos_I_user, trans_vel_I_user, trans_acc_I_user):
        
        # states
        
        self.roll = euler_angles[0]
        self.pitch = euler_angles[1]
        self.yaw = euler_angles[2]
        
        self.yaw_ref = yaw_ref
        self.yaw_ref_dot = yaw_ref_dot
        self.yaw_ref_ddot = yaw_ref_ddot
        
        self.angular_velocity = angular_vel
        self.angular_acceleration = angular_acc
        
        self.translational_position_in_I = trans_pos_NED
        self.translational_velocity_in_I = trans_vel_NED
        
        self.translational_position_in_I_user = trans_pos_I_user
        self.translational_velocity_in_I_user = trans_vel_I_user
        self.translational_acceleration_in_I_user = trans_acc_I_user
        
        # error calculations
        
        self.translational_position_error = self.translational_position_in_I - self.translational_position_in_I_user
        #self.angular_error = np.array([roll - roll_ref, pitch - pitch_ref, (((yaw - yaw_ref + PI) % (2*PI)) - PI)]).reshape(3,1)
        self.x_tran = np.append(self.translational_position_in_I, self.translational_velocity_in_I, axis=0)
        
