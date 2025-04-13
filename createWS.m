
clear all
close all
clc

%%
M = readmatrix('DATA_PID.csv'); M=M'; %M=M(2:end,1:74);

PID.PayAwa.data.time = M(:,1);
PID.PayAwa.data.simulation_time = M(:,2);
PID.PayAwa.data.translational_position_in_I = M(:,3:5);
PID.PayAwa.data.translational_velocity_in_I = M(:,6:8);
PID.PayAwa.data.roll = M(:,9);
PID.PayAwa.data.pitch = M(:,10);
PID.PayAwa.data.yaw = M(:,11);
PID.PayAwa.data.angular_velocity = M(:,12:14);
PID.PayAwa.data.roll_ref = M(:,15);
PID.PayAwa.data.pitch_ref = M(:,16);
PID.PayAwa.data.yaw_ref = M(:,17);
PID.PayAwa.data.roll_ref_dot = M(:,18);
PID.PayAwa.data.pitch_ref_dot = M(:,19);
PID.PayAwa.data.yaw_ref_dot = M(:,20);
PID.PayAwa.data.roll_ref_ddot = M(:,21);
PID.PayAwa.data.pitch_ref_ddot = M(:,22);
PID.PayAwa.data.yaw_ref_ddot = M(:,23);
PID.PayAwa.data.translational_position_in_I_user = M(:,24:26);
PID.PayAwa.data.translational_velocity_in_I_user = M(:,27:29);
PID.PayAwa.data.translational_acceleration_in_I_user = M(:,30:32);
PID.PayAwa.data.mu_x = M(:,33);
PID.PayAwa.data.mu_y = M(:,34);
PID.PayAwa.data.mu_z = M(:,35);
PID.PayAwa.data.u1 = M(:,36);
PID.PayAwa.data.u2 = M(:,37);
PID.PayAwa.data.u3 = M(:,38);
PID.PayAwa.data.u4 = M(:,39);
PID.PayAwa.data.thrust = M(:,40:47);

PID.PayAwa.data.yaw_ref = rem((PID.PayAwa.data.yaw_ref + pi), 2*pi) - pi;

PID.PayAwa.traj_track_error = [PID.PayAwa.data.translational_position_in_I PID.PayAwa.data.translational_velocity_in_I]...
    - [PID.PayAwa.data.translational_position_in_I_user PID.PayAwa.data.translational_velocity_in_I_user];

PID.PayAwa.traj_track_error_norm = vecnorm(PID.PayAwa.traj_track_error')';

PID.PayAwa.traj_track_error_L2norm = sqrt(cumtrapz(PID.PayAwa.data.time, PID.PayAwa.traj_track_error_norm.^2));

PID.PayAwa.pos_track_error = [PID.PayAwa.data.translational_position_in_I]...
    - [PID.PayAwa.data.translational_position_in_I_user];

PID.PayAwa.pos_track_error_norm = vecnorm(PID.PayAwa.pos_track_error')';


% %%
% M = readmatrix('data_PID_payload_unaware/value_1.csv'); M=M(4:end,:);
% 
% PID.PayUnawa.data.time = M(:,1);
% PID.PayUnawa.data.simulation_time = M(:,2);
% PID.PayUnawa.data.translational_position_in_I = M(:,3:5);
% PID.PayUnawa.data.translational_velocity_in_I = M(:,6:8);
% PID.PayUnawa.data.roll = M(:,9);
% PID.PayUnawa.data.pitch = M(:,10);
% PID.PayUnawa.data.yaw = M(:,11);
% PID.PayUnawa.data.angular_velocity = M(:,12:14);
% PID.PayUnawa.data.roll_ref = M(:,15);
% PID.PayUnawa.data.pitch_ref = M(:,16);
% PID.PayUnawa.data.yaw_ref = M(:,17);
% PID.PayUnawa.data.roll_ref_dot = M(:,18);
% PID.PayUnawa.data.pitch_ref_dot = M(:,19);
% PID.PayUnawa.data.yaw_ref_dot = M(:,20);
% PID.PayUnawa.data.roll_ref_ddot = M(:,21);
% PID.PayUnawa.data.pitch_ref_ddot = M(:,22);
% PID.PayUnawa.data.yaw_ref_ddot = M(:,23);
% PID.PayUnawa.data.translational_position_in_I_user = M(:,24:26);
% PID.PayUnawa.data.translational_velocity_in_I_user = M(:,27:29);
% PID.PayUnawa.data.translational_acceleration_in_I_user = M(:,30:32);
% PID.PayUnawa.data.mu_x = M(:,33);
% PID.PayUnawa.data.mu_y = M(:,34);
% PID.PayUnawa.data.mu_z = M(:,35);
% PID.PayUnawa.data.u1 = M(:,36);
% PID.PayUnawa.data.u2 = M(:,37);
% PID.PayUnawa.data.u3 = M(:,38);
% PID.PayUnawa.data.u4 = M(:,39);
% PID.PayUnawa.data.thrust = M(:,40:47);
% 
% PID.PayUnawa.data.yaw_ref = rem((PID.PayUnawa.data.yaw_ref + pi), 2*pi) - pi;
% 
% PID.PayUnawa.traj_track_error = [PID.PayUnawa.data.translational_position_in_I PID.PayUnawa.data.translational_velocity_in_I]...
%     - [PID.PayUnawa.data.translational_position_in_I_user PID.PayUnawa.data.translational_velocity_in_I_user];
% 
% PID.PayUnawa.traj_track_error_norm = vecnorm(PID.PayUnawa.traj_track_error')';
% 
% PID.PayUnawa.traj_track_error_L2norm = sqrt(cumtrapz(PID.PayUnawa.data.time, PID.PayUnawa.traj_track_error_norm.^2));
% 
% PID.PayUnawa.pos_track_error = [PID.PayUnawa.data.translational_position_in_I]...
%     - [PID.PayUnawa.data.translational_position_in_I_user];
% 
% PID.PayUnawa.pos_track_error_norm = vecnorm(PID.PayUnawa.pos_track_error')';

save('Workspace_1.mat');