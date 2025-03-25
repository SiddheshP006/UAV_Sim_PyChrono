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
    def __init__(self, gains_instance, ode_instance):
        print("PID controller CLASS")
        self.odeInputs = ode_instance  # Store ode_instance inside the class
        self.gains_instance = gains_instance

    def ode(self, t,y,ode_instance):
        """
        Defines the system of equations of the PID controller that need to be integrated.
        """
        # Y = np.zeros((10,1))
       
        # Access odeInputs variables directly
        self.odeInputs.translational_position_error = (
            self.odeInputs.translational_position_in_I - self.odeInputs.translational_position_in_I_user
        )

        # Reference variables for calculations
        
        state_phi_ref_diff = y[0:2]
        state_theta_ref_diff = y[2:4]
        integral_position_tracking = y[4:7]
        integral_angular_error = y[7:10]

        """self.mu_tran = self.gains_instance.mass_total_estimated * (
            -self.gains_instance.KP_tran * self.odeInputs.translational_position_error
            - self.gains_instance.KD_tran * (self.odeInputs.translational_velocity_in_I - self.odeInputs.translational_velocity_in_I_user)
            - self.gains_instance.KI_tran * integral_position_tracking
            + self.odeInputs.translational_acceleration_in_I_user
        )"""
        print(integral_position_tracking.shape)
        self.mu_tran = self.gains_instance.mass_total_estimated * (
            -self.gains_instance.KP_tran * self.odeInputs.translational_position_error
            - self.gains_instance.KD_tran * (self.odeInputs.translational_velocity_in_I - self.odeInputs.translational_velocity_in_I_user)
            - self.gains_instance.KI_tran * integral_position_tracking
            + self.odeInputs.translational_acceleration_in_I_user
            ).reshape(3,1)


        self.mu_x = self.mu_tran[0]
        self.mu_y = self.mu_tran[1]
        self.mu_z = self.mu_tran[2]
        
        self.u1 = math.sqrt(
            self.mu_x**2 + self.mu_y**2 + (self.gains_instance.mass_total_estimated * self.gains_instance.G_acc - self.mu_z) ** 2
        )

        calculation_var_A = -(1 / self.u1) * (self.mu_x * math.sin(self.odeInputs.yaw_ref) - self.mu_y * math.cos(self.odeInputs.yaw_ref))
        self.odeInputs.roll_ref = math.atan2(calculation_var_A, math.sqrt(1 - calculation_var_A**2))
        self.odeInputs.pitch_ref = math.atan2(
            -(self.mu_x * math.cos(self.odeInputs.yaw_ref) + self.mu_y * math.sin(self.odeInputs.yaw_ref)),
            (self.gains_instance.mass_total_estimated * self.gains_instance.G_acc - self.mu_z),
        )

        internal_state_differentiator_phi_ref_diff = self.gains_instance.A_phi_ref * state_phi_ref_diff + self.gains_instance.B_phi_ref * self.odeInputs.roll_ref
        internal_state_differentiator_theta_ref_diff = self.gains_instance.A_theta_ref * state_theta_ref_diff + self.gains_instance.B_theta_ref * self.odeInputs.pitch_ref

        self.odeInputs.roll_ref_dot = np.asarray(self.gains_instance.C_phi_ref * state_phi_ref_diff).item()
        self.odeInputs.pitch_ref_dot = np.asarray(self.gains_instance.C_theta_ref * state_theta_ref_diff).item()

        self.odeInputs.roll_ref_ddot = np.asarray(self.gains_instance.C_phi_ref * internal_state_differentiator_phi_ref_diff).item()
        self.odeInputs.pitch_ref_ddot = np.asarray(self.gains_instance.C_theta_ref * internal_state_differentiator_theta_ref_diff).item()

        self.angular_error = np.array(
            [
                self.odeInputs.roll - self.odeInputs.roll_ref,
                self.odeInputs.pitch - self.odeInputs.pitch_ref,
                (((self.odeInputs.yaw - self.odeInputs.yaw_ref + PI) % (2 * PI)) - PI),
            ]
        ).reshape(3, 1)

        self.angular_position_ref_dot = np.array(
            [self.odeInputs.roll_ref_dot, self.odeInputs.pitch_ref_dot, self.odeInputs.yaw_ref_dot]
        ).reshape(3, 1)
        self.angular_position_ref_ddot = np.array(
            [self.odeInputs.roll_ref_ddot, self.odeInputs.pitch_ref_ddot, self.odeInputs.yaw_ref_ddot]
        ).reshape(3, 1)

        Jacobian_matrix_inverse = np.matrix(
            [
                [1, (math.sin(self.odeInputs.roll) * math.sin(self.odeInputs.pitch)) / math.cos(self.odeInputs.pitch),
                 (math.cos(self.odeInputs.roll) * math.sin(self.odeInputs.pitch)) / math.cos(self.odeInputs.pitch)],
                [0, math.cos(self.odeInputs.roll), -math.sin(self.odeInputs.roll)],
                [0, math.sin(self.odeInputs.roll) / math.cos(self.odeInputs.pitch),
                 math.cos(self.odeInputs.roll) / math.cos(self.odeInputs.pitch)],
            ]
        )

        self.angular_position_dot = Jacobian_matrix_inverse * self.odeInputs.angular_velocity
        self.angular_error_dot = self.angular_position_dot - self.angular_position_ref_dot

        self.Moment = (
            np.cross(self.odeInputs.angular_velocity.ravel(), (self.gains_instance.I_matrix_estimated * self.odeInputs.angular_velocity).ravel()).reshape(3, 1)
            + self.gains_instance.I_matrix_estimated * (
                -self.gains_instance.KP_rot * self.angular_error
                - self.gains_instance.KD_rot * self.angular_error_dot
                - self.gains_instance.KI_rot * integral_angular_error
                + self.angular_position_ref_ddot
            ).reshape(3, 1)
        )

        self.u2 = self.Moment[0].item()
        self.u3 = self.Moment[1].item()
        self.u4 = self.Moment[2].item()
        print(self.odeInputs.roll)
        
        self.dy = np.zeros(10)  # Ensure dy is properly initialized
        self.dy[0:2] = internal_state_differentiator_phi_ref_diff.flatten()
        self.dy[2:4] = internal_state_differentiator_theta_ref_diff.flatten()
        self.dy[4:7] = self.odeInputs.translational_position_error.flatten()
        self.dy[7:10] = self.angular_error.ravel()  # Flatten for correct array structure

        return self.dy  # Ensure dy is returned
