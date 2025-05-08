# -*- coding: utf-8 -*-
"""
Created on Fri May  2 21:52:35 2025

@author: Melen
"""
# -*- coding: utf-8 -*-
"""
Created on Fri May  2 21:52:35 2025

@author: Melen
"""
import numpy as np
import math
from numpy import linalg as LA


class MRAC_with_BASELINE():
    def __init__(self,gains_instance,flight_params_instance,ode_instance,MRAC_Input):
        self.gains_instance = gains_instance
        self.params=flight_params_instance 
        self.odeInputs = ode_instance
        self.mrac_input= MRAC_Input


    def mrac_with_baseline_ode(self, t, y):

        state_phi_ref_diff = y[0:2]  # State of the differentiator for phi_ref (roll_ref)
        state_theta_ref_diff = y[2:4]  # State of the differentiator for theta_ref (pitch_ref)
        x_ref_tran = y[4:10]  # Reference model state
        integral_position_tracking_ref = y[10:13]  # Integral of ('translational_position_in_I_ref' - 'translational_position_in_I_user')
        K_hat_x_tran = y[13:31]  # \hat{K}_x (translational)
        K_hat_r_tran = y[31:40]  # \hat{K}_r (translational)
        Theta_hat_tran = y[40:58]  # \hat{\Theta} (translational)
        omega_ref = y[58:61]  # Reference model rotational dynamics
        K_hat_x_rot = y[61:70]  # \hat{K}_x (rotational)
        K_hat_r_rot = y[70:79]  # \hat{K}_r (rotational)
        Theta_hat_rot = y[79:97]  # \hat{\Theta} (rotational)
        integral_e_rot = y[97:100]  # Integral of 'e_rot' = (angular_velocity - omega_ref)
        e_transient_tran = y[100:106]  # Transient error dynamics translational (Two-layer)
        K_hat_g_tran = y[106:124]  # \hat{K}_g translational (Two-layer)
        e_transient_rot = y[124:127]  # Transient error dynamics rotational (Two-layer)
        K_hat_g_rot = y[127:136]  # \hat{K}_g (rotational)(Two-layer)
    
        K_hat_x_tran = np.matrix(K_hat_x_tran.reshape(6, 3))
        K_hat_r_tran = np.matrix(K_hat_r_tran.reshape(3, 3))
        Theta_hat_tran = np.matrix(Theta_hat_tran.reshape(6, 3))
        K_hat_x_rot = np.matrix(K_hat_x_rot.reshape(3, 3))
        K_hat_r_rot = np.matrix(K_hat_r_rot.reshape(3, 3))
        Theta_hat_rot = np.matrix(Theta_hat_rot.reshape(6, 3))
        K_hat_g_tran = np.matrix(K_hat_g_tran.reshape(6, 3))
        K_hat_g_rot = np.matrix(K_hat_g_rot.reshape(3, 3))
    
        self.e_tran = self.fx_tran - x_ref_tran
        self.e_rot = self.angular_velocity - omega_ref
        self.translational_position_in_I_ref = x_ref_tran[0:3]
    
        self.e_transient_tran_dot = self.A_transient_tran * e_transient_tran
        self.epsilon_tran = self.e_tran - e_transient_tran
        self.e_transient_rot_dot = self.A_transient_rot * e_transient_rot
        self.epsilon_rot = self.e_rot - e_transient_rot
    
        self.R3 = np.matrix([[math.cos(self.yaw), -math.sin(self.yaw), 0],
                             [math.sin(self.yaw), math.cos(self.yaw), 0],
                             [0, 0, 1]])
    
        self.R2 = np.matrix([[math.cos(self.pitch), 0, math.sin(self.pitch)],
                             [0, 1, 0],
                             [-math.sin(self.pitch), 0, math.cos(self.pitch)]])
    
        self.R1 = np.matrix([[1, 0, 0],
                             [0, math.cos(self.roll), -math.sin(self.roll)],
                             [0, math.sin(self.roll), math.cos(self.roll)]])
    
        self.R_from_loc_to_glob = self.R3 * self.R2 * self.R1
        self.R_from_glob_to_loc = self.R_from_loc_to_glob.transpose()
    
        self.Phi_adaptive_tran = -0.5 * LA.norm(self.R_from_glob_to_loc *self.translational_velocity_in_I) * (self.R_from_glob_to_loc * self.translational_velocity_in_I)
    
        Jacobian_matrix_inverse = np.matrix([[1, (math.sin(self.roll) * math.sin(self.pitch)) / math.cos(self.pitch), (math.cos(self.roll) * math.sin(self.pitch)) / math.cos(self.pitch)],
                                             [0, math.cos(self.roll), -math.sin(self.roll)],
                                             [0, math.sin(self.roll) / math.cos(self.pitch), math.cos(self.roll) / math.cos(self.pitch)]])
    
        self.angular_position_dot = Jacobian_matrix_inverse * self.angular_velocity
        self.roll_dot = self.angular_position_dot[0]
        self.pitch_dot = self.angular_position_dot[1]
        # yaw_dot = angular_position_dot[2]
    
        self.Jacobian_matrix_dot = np.matrix(np.zeros((3, 3)))
        self.Jacobian_matrix_dot[0, 2] = -math.cos(self.pitch) * self.pitch_dot
        self.Jacobian_matrix_dot[1, 1] = -math.sin(self.roll) * self.roll_dot
        self.Jacobian_matrix_dot[1, 2] = math.cos(self.roll) * math.cos(self.pitch) * self.roll_dot - math.sin(self.roll) * math.sin(self.pitch) * self.pitch_dot
        self.Jacobian_matrix_dot[2, 1] = -math.cos(self.roll) * self.roll_dot
        self.Jacobian_matrix_dot[2, 2] = -math.cos(self.pitch) * math.sin(self.roll) * self.roll_dot - math.cos(self.roll) * math.sin(self.pitch) * self.pitch_dot
    
        self.r_tran = self.mass_total_estimated * (
                -self.KI_tran * integral_position_tracking_ref + self.translational_acceleration_in_I_user + self.KP_tran * self.translational_position_in_I_user + self.KD_tran * self.translational_velocity_in_I_user)
    
        self.x_ref_tran_dot = self.A_ref_tran * x_ref_tran + self.B_ref_tran * self.r_tran
    
        self.mu_PD_baseline_tran = -self.mass_total_estimated * (
                self.KP_tran_PD_baseline * (self.translational_position_in_I - self.translational_position_in_I_ref) +
                self.KD_tran_PD_baseline * (self.translational_velocity_in_I - x_ref_tran[3:6]) - self.x_ref_tran_dot[3:6])
    
        self.Phi_adaptive_tran_augmented = np.matrix(np.block([[self.mu_PD_baseline_tran],
                                                               [self.Phi_adaptive_tran]]))
        self.Theta_tran_adaptive_bar_augmented = np.matrix(np.block([[np.identity(3)],
                                                                     [self.Theta_tran_adaptive_bar]]))
    
        self.mu_baseline_tran = self.K_x_tran_bar.T * self.x_tran + self.K_r_tran_bar.T * self.r_tran - self.Theta_tran_adaptive_bar_augmented.T * self.Phi_adaptive_tran_augmented + self.K_g_tran_bar.T * self.e_tran
        self.mu_adaptive_tran = K_hat_x_tran.T * self.x_tran + self.K_hat_r_tran.T * self.r_tran - Theta_hat_tran.T * self.Phi_adaptive_tran_augmented + K_hat_g_tran.T * self.e_tran
        self.mu_tran = self.mu_PD_baseline_tran + self.mu_baseline_tran + self.mu_adaptive_tran
    
        self.K_hat_x_tran_dot = -self.Gamma_x_tran * self.x_tran * self.epsilon_tran.T * self.P_tran_2Layer * self.B_tran
        self.K_hat_r_tran_dot = -self.Gamma_r_tran * self.r_tran * self.epsilon_tran.T * self.P_tran_2Layer * self.B_tran
        self.Theta_hat_tran_dot = self.Gamma_Theta_tran * self.Phi_adaptive_tran_augmented * self.epsilon_tran.T * self.P_tran_2Layer * self.B_tran
        self.K_hat_g_tran_dot = -self.Gamma_g_tran * self.e_tran * self.epsilon_tran.T * self.P_tran_2Layer * self.B_tran
    
        self.mu_x = self.mu_tran[0].item()
        self.mu_y = self.mu_tran[1].item()
        self.mu_z = self.mu_tran[2].item()
    
        self.u1 = math.sqrt(self.mu_x ** 2 + self.mu_y ** 2 + (self.mass_total_estimated * self.G_acc - self.mu_z) ** 2)
    
        calculation_var_A = -(1 / self.u1) * (self.mu_x * math.sin(self.yaw_ref) - self.mu_y * math.cos(self.yaw_ref))
        self.roll_ref = math.atan2(calculation_var_A, math.sqrt(1 - calculation_var_A ** 2))
    
        self.pitch_ref = math.atan2(-(self.mu_x * math.cos(self.yaw_ref) + self.mu_y * math.sin(self.yaw_ref)),
                                    (self.mass_total_estimated * self.G_acc - self.mu_z))
    
        internal_state_differentiator_phi_ref_diff = self.A_phi_ref * state_phi_ref_diff + self.B_phi_ref * self.roll_ref
        internal_state_differentiator_theta_ref_diff = self.A_theta_ref * state_theta_ref_diff + self.B_theta_ref * self.pitch_ref
    
        self.roll_ref_dot = np.asarray(self.C_phi_ref * state_phi_ref_diff).item()
        self.pitch_ref_dot = np.asarray(self.C_theta_ref * state_theta_ref_diff).item()
    
        self.roll_ref_ddot = np.asarray(self.C_phi_ref * internal_state_differentiator_phi_ref_diff).item()
        self.pitch_ref_ddot = np.asarray(self.C_theta_ref * internal_state_differentiator_theta_ref_diff).item()
    
        self.angular_position_ref_dot = np.array([self.roll_ref_dot, self.pitch_ref_dot, self.yaw_ref_dot]).reshape(3, 1)
        self.angular_position_ref_ddot = np.array([self.roll_ref_ddot, self.pitch_ref_ddot, self.yaw_ref_ddot]).reshape(3, 1)
    
        self.angular_error_dot = self.angular_position_dot - self.angular_position_ref_dot
    
        Jacobian_matrix = np.matrix([[1, 0, -math.sin(self.pitch)],
                                          [0, math.cos(self.roll), math.sin(self.roll) * math.cos(self.pitch)],
                                          [0, -math.sin(self.roll), math.cos(self.roll) * math.cos(self.pitch)]])
    
        self.omega_cmd = Jacobian_matrix * (-self.KP_rot * self.angular_error + self.angular_position_ref_dot)
        self.omega_cmd_dot = self.Jacobian_matrix_dot * (-self.KP_rot * self.angular_error + self.angular_position_ref_dot) + Jacobian_matrix * (
                -self.KP_rot * self.angular_error_dot + self.angular_position_ref_ddot)
    
        self.omega_ref_dot = -self.K_P_omega_ref * (omega_ref - self.omega_cmd) + self.omega_cmd_dot
    
        self.r_rot = self.K_P_omega_ref * self.omega_cmd + self.omega_cmd_dot
    
        self.Phi_adaptive_rot = np.array([[self.angular_velocity[1].item() * self.angular_velocity[2].item()],
                                          [self.angular_velocity[0].item() * self.angular_velocity[2].item()],
                                          [self.angular_velocity[0].item() * self.angular_velocity[1].item()]])
    
        self.Moment_baseline_PI = -self.I_matrix_estimated * (
                self.KP_rot_PI_baseline * self.e_rot + self.KI_rot_PI_baseline * integral_e_rot - self.omega_ref_dot)
    
        self.Phi_adaptive_rot_augmented = np.matrix(np.block([[self.Moment_baseline_PI],
                                                              [self.Phi_adaptive_rot]]))
    
        self.K_hat_x_rot_dot = -self.Gamma_x_rot * self.angular_velocity * self.epsilon_rot.T * self.P_rot_2Layer * self.B_rot
        self.K_hat_r_rot_dot = -self.Gamma_r_rot * self.r_rot * self.epsilon_rot.T * self.P_rot_2Layer * self.B_rot
        self.Theta_hat_rot_dot = self.Gamma_Theta_rot * self.Phi_adaptive_rot_augmented * self.epsilon_rot.T * self.P_rot_2Layer * self.B_rot
        self.K_hat_g_rot_dot = -self.Gamma_g_rot * self.e_rot * self.epsilon_rot.T * self.P_rot_2Layer * self.B_rot
    
        self.seMoment_baseline = np.cross(self.angular_velocity.ravel(), (self.I_matrix_estimated * self.angular_velocity).ravel()).reshape(3, 1)
        self.Moment_adaptive = K_hat_x_rot.T * self.angular_velocity + K_hat_r_rot.T * self.r_rot - Theta_hat_rot.T * self.Phi_adaptive_rot_augmented + K_hat_g_rot.T * self.e_rot
    
        self.Moment = self.Moment_baseline_PI + self.Moment_baseline + self.Moment_adaptive
    
        self.u2 = self.Moment[0].item()
        self.u3 = self.Moment[1].item()
        self.u4 = self.Moment[2].item()
    
        self.dy[0:2] = internal_state_differentiator_phi_ref_diff
        self.dy[2:4] = internal_state_differentiator_theta_ref_diff
        self.dy[4:10] = self.x_ref_tran_dot
        self.dy[10:13] = self.translational_position_in_I_ref - self.translational_position_in_I_user
        self.dy[13:31] = self.K_hat_x_tran_dot.reshape(18, 1)
        self.dy[31:40] = self.K_hat_r_tran_dot.reshape(9, 1)
        self.dy[40:58] = self.Theta_hat_tran_dot.reshape(18, 1)
        self.dy[58:61] = self.omega_ref_dot
        self.dy[61:70] = self.K_hat_x_rot_dot.reshape(9, 1)
        self.dy[70:79] = self.K_hat_r_rot_dot.reshape(9, 1)
        self.dy[79:97] = self.Theta_hat_rot_dot.reshape(18, 1)
        self.dy[97:100] = self.angular_velocity - omega_ref
        self.dy[100:106] = self.e_transient_tran_dot
        self.dy[106:124] = self.K_hat_g_tran_dot.reshape(18, 1)
        self.dy[124:127] = self.e_transient_rot_dot
        self.dy[127:136] = self.K_hat_g_rot_dot.reshape(9, 1)
    
        return self.dy
