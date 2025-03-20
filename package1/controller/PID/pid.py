# -*- coding: utf-8 -*-
"""
Created on Sun Feb 18 21:44:13 2024

@author: grem6
"""

import math
import numpy as np  
from package1.functions import *

PI = math.pi
#dddddd

class PID_controller:
  def __init__(self, gains_instance):
    print("PID controller CLASS")
    self.G_acc = gains_instance.G_acc
    self.mass_total_estimated = gains_instance.mass_total_estimated
    self.I_matrix_estimated = gains_instance.I_matrix_estimated
    self.A_phi_ref = gains_instance.A_phi_ref
    self.B_phi_ref = gains_instance.B_phi_ref
    self.C_phi_ref = gains_instance.C_phi_ref
    self.A_theta_ref = gains_instance.A_theta_ref
    self.B_theta_ref = gains_instance.B_theta_ref
    self.C_theta_ref = gains_instance.C_theta_ref     
    self.KP_tran = gains_instance.KP_tran
    self.KD_tran = gains_instance.KD_tran
    self.KI_tran = gains_instance.KI_tran
    self.KP_rot = gains_instance.KP_rot
    self.KD_rot = gains_instance.KD_rot
    self.KI_rot = gains_instance.KI_rot
    self.gains=gains_instance

  def ode(self, t, y,euler_angles, yaw_ref, yaw_ref_dot, yaw_ref_ddot, angular_vel, angular_acc, trans_pos_NED, trans_vel_NED, trans_pos_I_user, trans_vel_I_user, trans_acc_I_user):
      """
      This function defines the system of equations of the PID CONTROLLER that need to be integrated 
  
      """
      

      self.roll = euler_angles.x
      self.pitch = euler_angles.y
      self.yaw = euler_angles.z
      
      self.yaw_ref = yaw_ref
      self.yaw_ref_dot = yaw_ref_dot
      self.yaw_ref_ddot = yaw_ref_ddot
      
      self.angular_velocity = np.array(chvector_to_list(angular_vel)).reshape(3,1)
      self.angular_acceleration = np.array(chvector_to_list(angular_acc)).reshape(3,1)
      
      self.translational_position_in_I = np.array(chvector_to_list(pos_pixhawk_LOC_to_GLOB_NED)).reshape(3,1)
      self.translational_velocity_in_I = np.array(chvector_to_list(vel_pixhawk_LOC_to_GLOB_NED)).reshape(3,1)
      
      self.translational_position_in_I_user = trans_pos_I_user
      self.translational_velocity_in_I_user = trans_vel_I_user
      self.translational_acceleration_in_I_user = trans_acc_I_user
      
      # error calculations
      
      self.translational_position_error = self.translational_position_in_I - trans_pos_I_user
      #self.angular_error = np.array([roll - roll_ref, pitch - pitch_ref, (((yaw - yaw_ref + PI) % (2*PI)) - PI)]).reshape(3,1)
      self.x_tran = np.append(self.translational_position_in_I, self.translational_velocity_in_I, axis=0)
     
      
      # error calcs
      
      self.translational_position_error = odeInput.translational_position_error
      #self.angular_error = odeInput.angular_error
      
      state_phi_ref_diff = y[0:2]
      state_theta_ref_diff = y[2:4]
      integral_position_tracking = y[4:7]
      integral_angular_error = y[7:10]
      
      
      self.mu_tran = self.mass_total_estimated*(- self.KP_tran * self.translational_position_error 
                                           - self.KD_tran * (self.translational_velocity_in_I - self.translational_velocity_in_I_user)
                                           - self.KI_tran * integral_position_tracking
                                           + self.translational_acceleration_in_I_user).reshape(3,1)
      
      self.mu_x = self.mu_tran[0].item()
      self.mu_y = self.mu_tran[1].item()
      self.mu_z = self.mu_tran[2].item()
      
      self.u1 = math.sqrt(self.mu_x ** 2 + self.mu_y ** 2 + (self.mass_total_estimated * self.G_acc - self.mu_z) ** 2)
      
      calculation_var_A = -(1/self.u1) * (self.mu_x * math.sin(self.yaw_ref) - self.mu_y * math.cos(self.yaw_ref))
      self.roll_ref = math.atan2(calculation_var_A, math.sqrt(1 - calculation_var_A ** 2))
      
      self.pitch_ref = math.atan2(-(self.mu_x * math.cos(self.yaw_ref) + self.mu_y * math.sin(self.yaw_ref)), (self.mass_total_estimated * self.G_acc - self.mu_z))
      
      internal_state_differentiator_phi_ref_diff = self.A_phi_ref * state_phi_ref_diff + self.B_phi_ref*self.roll_ref
      internal_state_differentiator_theta_ref_diff = self.A_theta_ref * state_theta_ref_diff + self.B_theta_ref*self.pitch_ref

      self.roll_ref_dot = np.asarray(self.C_phi_ref*state_phi_ref_diff).item()
      self.pitch_ref_dot = np.asarray(self.C_theta_ref*state_theta_ref_diff).item()
      
      self.roll_ref_ddot = np.asarray(self.C_phi_ref*internal_state_differentiator_phi_ref_diff).item()
      self.pitch_ref_ddot = np.asarray(self.C_theta_ref*internal_state_differentiator_theta_ref_diff).item()
      
      self.angular_error = np.array([self.roll - self.roll_ref, self.pitch - self.pitch_ref, (((self.yaw - self.yaw_ref + PI) % (2*PI)) - PI)]).reshape(3,1)
      
      self.angular_position_ref_dot = np.array([self.roll_ref_dot, self.pitch_ref_dot, self.yaw_ref_dot]).reshape(3,1)
      self.angular_position_ref_ddot = np.array([self.roll_ref_ddot, self.pitch_ref_ddot, self.yaw_ref_ddot]).reshape(3,1)
      
      Jacobian_matrix_inverse = np.matrix([[1, (math.sin(self.roll)*math.sin(self.pitch))/math.cos(self.pitch), (math.cos(self.roll)*math.sin(self.pitch))/math.cos(self.pitch)],
                                           [0,                                             math.cos(self.roll),                                            -math.sin(self.roll)],
                                           [0,                        math.sin(self.roll)/math.cos(self.pitch),                       math.cos(self.roll)/math.cos(self.pitch)]])
  
      self.angular_position_dot = Jacobian_matrix_inverse * self.angular_velocity
      self.angular_error_dot = self.angular_position_dot - self.angular_position_ref_dot
      
      self.Moment = np.cross(self.angular_velocity.ravel(), (self.I_matrix_estimated * self.angular_velocity).ravel()).reshape(3,1) 
      + self.I_matrix_estimated * (- self.KP_rot * self.angular_error
                                   - self.KD_rot * self.angular_error_dot
                                   - self.KI_rot * integral_angular_error
                                   + self.angular_position_ref_ddot).reshape(3,1)
      
      
      
      self.u2 = self.Moment[0].item()
      self.u3 = self.Moment[1].item()
      self.u4 = self.Moment[2].item()
      
      self.dy[0:2] = internal_state_differentiator_phi_ref_diff
      self.dy[2:4] = internal_state_differentiator_theta_ref_diff
      self.dy[4:7] = self.translational_position_error
      self.dy[7:10] = self.angular_error
  
      #return np.array(self.dy)
      return self.dy
