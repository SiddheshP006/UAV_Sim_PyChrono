
clear all
close all
clc

%%
M = readmatrix('DATA_PID.csv'); M=M'; %M=M(2:end,1:74);

PID_Modular.PayAwa.data.time = M(:,1);
PID_Modular.PayAwa.data.simulation_time = M(:,2);
PID_Modular.PayAwa.data.translational_position_in_I = M(:,3:5);
PID_Modular.PayAwa.data.translational_velocity_in_I = M(:,6:8);
PID_Modular.PayAwa.data.roll = M(:,9);
PID_Modular.PayAwa.data.pitch = M(:,10);
PID_Modular.PayAwa.data.yaw = M(:,11);
PID_Modular.PayAwa.data.angular_velocity = M(:,12:14);
PID_Modular.PayAwa.data.roll_ref = M(:,15);
PID_Modular.PayAwa.data.pitch_ref = M(:,16);
PID_Modular.PayAwa.data.yaw_ref = M(:,17);
PID_Modular.PayAwa.data.roll_ref_dot = M(:,18);
PID_Modular.PayAwa.data.pitch_ref_dot = M(:,19);
PID_Modular.PayAwa.data.yaw_ref_dot = M(:,20);
PID_Modular.PayAwa.data.roll_ref_ddot = M(:,21);
PID_Modular.PayAwa.data.pitch_ref_ddot = M(:,22);
PID_Modular.PayAwa.data.yaw_ref_ddot = M(:,23);
PID_Modular.PayAwa.data.translational_position_in_I_user = M(:,24:26);
PID_Modular.PayAwa.data.translational_velocity_in_I_user = M(:,27:29);
PID_Modular.PayAwa.data.translational_acceleration_in_I_user = M(:,30:32);
PID_Modular.PayAwa.data.mu_x = M(:,33);
PID_Modular.PayAwa.data.mu_y = M(:,34);
PID_Modular.PayAwa.data.mu_z = M(:,35);
PID_Modular.PayAwa.data.u1 = M(:,36);
PID_Modular.PayAwa.data.u2 = M(:,37);
PID_Modular.PayAwa.data.u3 = M(:,38);
PID_Modular.PayAwa.data.u4 = M(:,39);
PID_Modular.PayAwa.data.thrust = M(:,40:47);

PID_Modular.PayAwa.data.yaw_ref = rem((PID_Modular.PayAwa.data.yaw_ref + pi), 2*pi) - pi;

PID_Modular.PayAwa.traj_track_error = [PID_Modular.PayAwa.data.translational_position_in_I PID_Modular.PayAwa.data.translational_velocity_in_I]...
    - [PID_Modular.PayAwa.data.translational_position_in_I_user PID_Modular.PayAwa.data.translational_velocity_in_I_user];

PID_Modular.PayAwa.traj_track_error_norm = vecnorm(PID_Modular.PayAwa.traj_track_error')';

PID_Modular.PayAwa.traj_track_error_L2norm = sqrt(cumtrapz(PID_Modular.PayAwa.data.time, PID_Modular.PayAwa.traj_track_error_norm.^2));

PID_Modular.PayAwa.pos_track_error = [PID_Modular.PayAwa.data.translational_position_in_I]...
    - [PID_Modular.PayAwa.data.translational_position_in_I_user];

PID_Modular.PayAwa.pos_track_error_norm = vecnorm(PID_Modular.PayAwa.pos_track_error')';


% %%
% M = readmatrix('data_PID_payload_unaware/value_1.csv'); M=M(4:end,:);
% 
% PID_Modular.PayUnawa.data.time = M(:,1);
% PID_Modular.PayUnawa.data.simulation_time = M(:,2);
% PID_Modular.PayUnawa.data.translational_position_in_I = M(:,3:5);
% PID_Modular.PayUnawa.data.translational_velocity_in_I = M(:,6:8);
% PID_Modular.PayUnawa.data.roll = M(:,9);
% PID_Modular.PayUnawa.data.pitch = M(:,10);
% PID_Modular.PayUnawa.data.yaw = M(:,11);
% PID_Modular.PayUnawa.data.angular_velocity = M(:,12:14);
% PID_Modular.PayUnawa.data.roll_ref = M(:,15);
% PID_Modular.PayUnawa.data.pitch_ref = M(:,16);
% PID_Modular.PayUnawa.data.yaw_ref = M(:,17);
% PID_Modular.PayUnawa.data.roll_ref_dot = M(:,18);
% PID_Modular.PayUnawa.data.pitch_ref_dot = M(:,19);
% PID_Modular.PayUnawa.data.yaw_ref_dot = M(:,20);
% PID_Modular.PayUnawa.data.roll_ref_ddot = M(:,21);
% PID_Modular.PayUnawa.data.pitch_ref_ddot = M(:,22);
% PID_Modular.PayUnawa.data.yaw_ref_ddot = M(:,23);
% PID_Modular.PayUnawa.data.translational_position_in_I_user = M(:,24:26);
% PID_Modular.PayUnawa.data.translational_velocity_in_I_user = M(:,27:29);
% PID_Modular.PayUnawa.data.translational_acceleration_in_I_user = M(:,30:32);
% PID_Modular.PayUnawa.data.mu_x = M(:,33);
% PID_Modular.PayUnawa.data.mu_y = M(:,34);
% PID_Modular.PayUnawa.data.mu_z = M(:,35);
% PID_Modular.PayUnawa.data.u1 = M(:,36);
% PID_Modular.PayUnawa.data.u2 = M(:,37);
% PID_Modular.PayUnawa.data.u3 = M(:,38);
% PID_Modular.PayUnawa.data.u4 = M(:,39);
% PID_Modular.PayUnawa.data.thrust = M(:,40:47);
% 
% PID_Modular.PayUnawa.data.yaw_ref = rem((PID_Modular.PayUnawa.data.yaw_ref + pi), 2*pi) - pi;
% 
% PID_Modular.PayUnawa.traj_track_error = [PID_Modular.PayUnawa.data.translational_position_in_I PID_Modular.PayUnawa.data.translational_velocity_in_I]...
%     - [PID_Modular.PayUnawa.data.translational_position_in_I_user PID_Modular.PayUnawa.data.translational_velocity_in_I_user];
% 
% PID_Modular.PayUnawa.traj_track_error_norm = vecnorm(PID_Modular.PayUnawa.traj_track_error')';
% 
% PID_Modular.PayUnawa.traj_track_error_L2norm = sqrt(cumtrapz(PID_Modular.PayUnawa.data.time, PID_Modular.PayUnawa.traj_track_error_norm.^2));
% 
% PID_Modular.PayUnawa.pos_track_error = [PID_Modular.PayUnawa.data.translational_position_in_I]...
%     - [PID_Modular.PayUnawa.data.translational_position_in_I_user];
% 
% PID_Modular.PayUnawa.pos_track_error_norm = vecnorm(PID_Modular.PayUnawa.pos_track_error')';

save('Workspace_1_modular.mat');