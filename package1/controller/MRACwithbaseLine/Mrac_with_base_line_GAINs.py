# -*- coding: utf-8 -*-
"""
Created on Fri May  2 20:51:45 2025

@author: Melen
"""
import numpy as np

import numpy as np
from scipy import linalg
from numpy import linalg as LA




class MracWithBaseLineGains:
    def __init__(self, mass_total_estimated,
                 air_density_estimated,
                 surface_area_estimated,
                 drag_coefficient_matrix_estimated):
        # ----------------------------------------------------------------
        #                     Baseline Parameters
        # ----------------------------------------------------------------

        # Translational baseline parameters
        self.KP_tran = np.matrix(np.diag([5, 5, 6]))
        self.KD_tran = np.matrix(np.diag([8, 8, 3]))
        self.KI_tran = np.matrix(np.diag([1, 1, 0.1]))

        self.KP_tran_PD_baseline = np.matrix(np.diag([5, 5, 6]))
        self.KD_tran_PD_baseline = np.matrix(np.diag([8, 8, 3]))

        # Rotational baseline gains
        self.KP_rot = np.matrix(1e2 * np.diag([1, 1, 0.5]))

        self.KP_rot_PI_baseline = np.matrix(40 * np.diag([1, 1, 1]))
        self.KD_rot_PI_baseline = np.matrix(np.zeros((3, 3)))
        self.KI_rot_PI_baseline = np.matrix(0.1 * np.diag([1, 1, 0.5]))

        self.K_P_omega_ref = np.matrix(0.15 * np.diag([5, 5, 10]))

        # ----------------------------------------------------------------
        #                   Translational Parameters MRAC
        # ----------------------------------------------------------------

        self.A_tran = np.block([
            [np.zeros((3, 3)), np.identity(3)],
            [np.zeros((3, 3)), np.zeros((3, 3))]
        ])

        self.B_tran = np.matrix(np.block([
            [np.zeros((3, 3))],
            [np.identity(3)]
        ]))

        self.A_tran_bar = self.A_tran.copy()
        self.Lambda_bar = (1 / mass_total_estimated) * np.identity(3)
        self.Theta_tran_adaptive_bar = (air_density_estimated
                                        * surface_area_estimated
                                        * drag_coefficient_matrix_estimated)

        self.A_ref_tran = np.block([
            [np.zeros((3, 3)), np.identity(3)],
            [-self.KP_tran,    -self.KD_tran]
        ])

        self.B_ref_tran = np.matrix(np.block([
            [np.zeros((3, 3))],
            [(1 / mass_total_estimated) * np.identity(3)]
        ]))

        # Adaptive‑law rate matrices (constant design choices)
        self.Gamma_x_tran = np.matrix(5e2 * np.diag([1, 1, 10, 1, 1, 10]))
        self.Gamma_r_tran = np.matrix(np.diag([1, 1, 1]))
        self.Gamma_Theta_tran = np.matrix(np.diag([1, 1, 1, 1, 1, 1]))

        # Lyapunov design
        self.Q_tran = np.matrix(4e-3 * np.diag([1, 1, 20, 1, 1, 20]))
        self.P_tran = np.matrix(
            linalg.solve_continuous_lyapunov(self.A_ref_tran.T, -self.Q_tran))

        # Matching‑condition baseline gains
        self.K_x_tran_bar = (LA.pinv(self.B_tran @ self.Lambda_bar)
                             @ (self.A_ref_tran - self.A_tran_bar)).T
        self.K_r_tran_bar = (LA.pinv(self.B_tran @ self.Lambda_bar)
                             @ self.B_ref_tran).T

        # ----------------------------------------------------------------
        #                   Rotational Parameters MRAC
        # ----------------------------------------------------------------

        self.A_rot = np.matrix(np.zeros((3, 3)))
        self.B_rot = np.matrix(np.eye(3))

        self.A_ref_rot = -self.K_P_omega_ref
        self.B_ref_rot = np.matrix(np.eye(3))

        self.Q_rot = np.matrix(7e-3 * np.diag([1, 1, 2]))
        self.P_rot = np.matrix(
            linalg.solve_continuous_lyapunov(self.A_ref_rot.T, -self.Q_rot))

        self.Gamma_x_rot = np.matrix(1e1 * np.diag([1, 1, 10]))
        self.Gamma_r_rot = np.matrix(1e-4 * np.diag([1, 1, 1]))
        self.Gamma_Theta_rot = np.matrix(np.diag([1, 1, 1, 1, 1, 1]))
