# -*- coding: utf-8 -*-
"""
Created on Wed Apr  9 21:51:46 2025

@author: Melen
"""

import datetime
import os
import numpy as np  # Needed if you use np.floating

# Initialize global counter if not yet defined
if 'log_counter' not in globals():
    log_counter = 1

# Set the path to your log file
log_filename = "PID_LOG.log"

# Define your column headers (match your PID vector structure)
column_titles = [
    "Initial timestamp",
    "Current time [s]",
    "User-defined position x [m]",
    "User-defined position y [m]",
    "User-defined position z [m]",
    "User-defined velocity x [m/s]",
    "User-defined velocity y [m/s]",
    "User-defined velocity z [m/s]",
    "User-defined acceleration x [m/s^2]",
    "User-defined acceleration y [m/s^2]",
    "User-defined acceleration z [m/s^2]",
    "User-defined yaw [rad]",
    "User-defined yaw_dot [rad/s]",
    "User-defined yaw_dot_dot [rad/s^2]",
    "Odometry time [s]",
    "Position x [m]",
    "Position y [m]",
    "Position z [m]",
    "Quaternion q0 [-]",
    "Quaternion q1 [-]",
    "Quaternion q2 [-]",
    "Quaternion q3 [-]",
    "Velocity x [m/s]",
    "Velocity y [m/s]",
    "Velocity z [m/s]",
    "Angular velocity x [rad/s]",
    "Angular velocity y [rad/s]",
    "Angular velocity z [rad/s]",
    "Roll [rad]",
    "Pitch [rad]",
    "Yaw [rad]",
    "Algorithm execution time [us]",
    "Mu translational raw x [N]",
    "Mu translational raw y [N]",
    "Mu translational raw z [N]",
    "Mu translational x [N]",
    "Mu translational y [N]",
    "Mu translational z [N]",
    "U control input U1 [N]",
    "U control input U2 [Nm]",
    "U control input U3 [Nm]",
    "U control input U4 [Nm]",
    "Roll desired [rad]",
    "Pitch desired [rad]",
    "Roll_dot desired [rad/s]",
    "Pitch_dot desired [rad/s]",
    "Roll_dot_dot desired [rad/s^2]",
    "Pitch_dot_dot desired [rad/s^2]",
    "Roll_dot [rad/s]",
    "Pitch_dot [rad/s]",
    "Yaw_dot [rad/s]",
    "Thrust motors QUADCOPTER T1 [N]",
    "Thrust motors QUADCOPTER T2 [N]",
    "Thrust motors QUADCOPTER T3 [N]",
    "Thrust motors QUADCOPTER T4 [N]",
    "Thrust motors normalized QUADCOPTER T1 [-]",
    "Thrust motors normalized QUADCOPTER T2 [-]",
    "Thrust motors normalized QUADCOPTER T3 [-]",
    "Thrust motors normalized QUADCOPTER T4 [-]",
    "Thrust motors X8COPTER T1 [N]",
    "Thrust motors X8COPTER T2 [N]",
    "Thrust motors X8COPTER T3 [N]",
    "Thrust motors X8COPTER T4 [N]",
    "Thrust motors X8COPTER T5 [N]",
    "Thrust motors X8COPTER T6 [N]",
    "Thrust motors X8COPTER T7 [N]",
    "Thrust motors X8COPTER T8 [N]",
    "Thrust motors normalized X8COPTER T1 [-]",
    "Thrust motors normalized X8COPTER T2 [-]",
    "Thrust motors normalized X8COPTER T3 [-]",
    "Thrust motors normalized X8COPTER T4 [-]",
    "Thrust motors normalized X8COPTER T5 [-]",
    "Thrust motors normalized X8COPTER T6 [-]",
    "Thrust motors normalized X8COPTER T7 [-]",
    "Thrust motors normalized X8COPTER T8 [-]",
    "Roll error [rad]",
    "Pitch error [rad]",
    "Yaw error [rad]",
    "Outer loop Proportional term x [-]",
    "Outer loop Proportional term y [-]",
    "Outer loop Proportional term z [-]",
    "Outer loop Integral term x [-]",
    "Outer loop Integral term y [-]",
    "Outer loop Integral term z [-]",
    "Outer loop Derivative term x [-]",
    "Outer loop Derivative term y [-]",
    "Outer loop Derivative term z [-]",
    "Outer loop Dynamic Inversion term x [-]",
    "Outer loop Dynamic Inversion term y [-]",
    "Outer loop Dynamic Inversion term z [-]",
    "Inner loop Proportional term x [-]",
    "Inner loop Proportional term y [-]",
    "Inner loop Proportional term z [-]",
    "Inner loop Integral term x [-]",
    "Inner loop Integral term y [-]",
    "Inner loop Integral term z [-]",
    "Inner loop Derivative term x [-]",
    "Inner loop Derivative term y [-]",
    "Inner loop Derivative term z [-]",
    "Inner loop Dynamic Inversion term x [-]",
    "Inner loop Dynamic Inversion term y [-]",
    "Inner loop Dynamic Inversion term z [-]"
]


def log_data_vector_with_header(data_vector, file_name=log_filename):
    global log_counter

    # Get formatted time and header prefix
    now = datetime.datetime.now().strftime("[%Y-%m-%d %H:%M:%S.%f]")
    header_prefix = f"{now} [0x00007f006e7d1f80] [info] [0x00010092]"

    # Write column header once
    if log_counter == 1 and (not os.path.exists(file_name) or os.stat(file_name).st_size == 0):
        with open(file_name, "a") as f:
            f.write(f"{header_prefix} [{log_counter}] {', '.join(column_titles)}\n")
        log_counter += 1

    # Format the data line
    data_str = ", ".join([f"{x:.6e}" if isinstance(x, float) or isinstance(x, np.floating) else str(x)
                          for x in data_vector.flatten()])
    with open(file_name, "a") as f:
        f.write(f"{header_prefix} [{log_counter}] {data_str}\n")

    log_counter += 1
