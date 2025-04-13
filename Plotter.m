

clear all
close all
clc

% load('Workspace');
load('Workspace_1.mat');

% Overall properties
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
set(groot, 'defaultAxesFontSize',20);


font_size = 20;
font_size_title = 22;

%% Plot thrust vs time PID Unaware of payload
set(figure,'Color','white','WindowState','maximized')
plot(PID.PayAwa.data.time,sum(PID.PayAwa.data.thrust,2),'b-','LineWidth',2)
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Total thrust [N]','interpreter','latex','fontsize',font_size)
title('PID','interpreter','latex','fontsize',font_size_title)
hold off
axis tight
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
legend('$u_1(t)$','Motor failure'); 
% str={'Motor failure'};
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.655, 0.75, 0.2, 0.2], 'EdgeColor', 'none');




% axis ([min(PID.PayAwa.data.time), max(PID.PayAwa.data.time), min(sum(PID.PayAwa.data.thrust,2)),max(sum(PID.PayAwa.data.thrust,2))])

set(figure,'Color','white','WindowState','maximized')
subplot(2,1,1)
plot(PID.PayAwa.data.time,PID.PayAwa.data.thrust(:,1),'b-','LineWidth',2)
hold on
plot(PID.PayAwa.data.time,PID.PayAwa.data.thrust(:,2),'r-.','LineWidth',2)
plot(PID.PayAwa.data.time,PID.PayAwa.data.thrust(:,3),'g--','LineWidth',2)
plot(PID.PayAwa.data.time,PID.PayAwa.data.thrust(:,4),'k-','LineWidth',0.5)
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
legend('$T_1(t)$','$T_2(t)$','$T_3(t)$','$T_4(t)$','');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Thrust motor $i$ [N]','interpreter','latex','fontsize',font_size)
title('PID','interpreter','latex','fontsize',font_size_title)
hold off
axis tight
str={'Motor failure'};
annotation('textbox','interpreter','latex','String',str,'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.655, 0.75, 0.2, 0.2], 'EdgeColor', 'none');

subplot(2,1,2)
plot(PID.PayAwa.data.time,PID.PayAwa.data.thrust(:,5),'b-','LineWidth',2)
hold on
plot(PID.PayAwa.data.time,PID.PayAwa.data.thrust(:,6),'r-.','LineWidth',2)
plot(PID.PayAwa.data.time,PID.PayAwa.data.thrust(:,7),'g--','LineWidth',2)
plot(PID.PayAwa.data.time,PID.PayAwa.data.thrust(:,8),'k-','LineWidth',0.5)
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
legend('$T_5(t)$','$T_6(t)$','$T_7(t)$','$T_8(t)$','Motor failure');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Thrust motor $i$ [N]','interpreter','latex','fontsize',font_size)
hold off
axis tight
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.655, 0.275, 0.2, 0.2], 'EdgeColor', 'none');

%--------------------------------------------------------------------------
%% Plot angles vs. time PID Unaware of payload
set(figure,'Color','white','WindowState','maximized')
subplot(3,1,1)
plot(PID.PayAwa.data.time,rad2deg(PID.PayAwa.data.roll),'b-','LineWidth',2)
hold on
plot(PID.PayAwa.data.time,rad2deg(PID.PayAwa.data.roll_ref),'r-.','LineWidth',2)
legend('$$\phi(t)$$','$$\phi_{\rm ref}(t)$$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Roll [deg]','interpreter','latex','fontsize',font_size)
title('PID','interpreter','latex','fontsize',font_size_title)
hold off
axis tight
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.655, 0.75, 0.2, 0.2], 'EdgeColor', 'none');

subplot(3,1,2)
plot(PID.PayAwa.data.time,rad2deg(PID.PayAwa.data.pitch),'b-','LineWidth',2)
hold on
plot(PID.PayAwa.data.time,rad2deg(PID.PayAwa.data.pitch_ref),'r-.','LineWidth',2)
legend('$$\theta(t)$$','$$\theta_{\rm ref}(t)$$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Pitch [deg]','interpreter','latex','fontsize',font_size)
hold off
axis tight
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.655, 0.45, 0.2, 0.2], 'EdgeColor', 'none');

subplot(3,1,3)
plot(PID.PayAwa.data.time,rad2deg(PID.PayAwa.data.yaw),'b-','LineWidth',2)
hold on
plot(PID.PayAwa.data.time,rad2deg(PID.PayAwa.data.yaw_ref),'r-.','LineWidth',2)
legend('$$\psi(t)$$','$$\psi_{\rm ref}(t)$$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Yaw [deg]','interpreter','latex','fontsize',font_size)
hold off
axis tight
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.655, 0.155, 0.2, 0.2], 'EdgeColor', 'none');

%--------------------------------------------------------------------------
%% Plot position vs time PID Unaware of payload

set(figure,'Color','white','WindowState','maximized')
subplot(3,1,1)
plot(PID.PayAwa.data.time,PID.PayAwa.data.translational_position_in_I(:,1),'r-','LineWidth',2)
hold on
plot(PID.PayAwa.data.time,PID.PayAwa.data.translational_position_in_I_user(:,1),'b-.','LineWidth',2)
legend('$r_x(t)$','$r_{\rm user,x}(t)$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('X position [m]','interpreter','latex','fontsize',font_size)
title('PID','interpreter','latex','fontsize',font_size_title)
hold off
axis tight
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.655, 0.75, 0.2, 0.2], 'EdgeColor', 'none');


subplot(3,1,2)
plot(PID.PayAwa.data.time,PID.PayAwa.data.translational_position_in_I(:,2),'r-','LineWidth',2)
hold on
plot(PID.PayAwa.data.time,PID.PayAwa.data.translational_position_in_I_user(:,2),'b-.','LineWidth',2)
legend('$r_y(t)$','$r_{\rm user,y}(t)$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Y position [m]','interpreter','latex','fontsize',font_size)
hold off
axis tight
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.655, 0.45, 0.2, 0.2], 'EdgeColor', 'none');

subplot(3,1,3)
plot(PID.PayAwa.data.time,-PID.PayAwa.data.translational_position_in_I(:,3),'r-','LineWidth',2)
hold on
plot(PID.PayAwa.data.time,-PID.PayAwa.data.translational_position_in_I_user(:,3),'b-.','LineWidth',2)
legend('$r_z(t)$','$r_{\rm user,z}(t)$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Z position [m]','interpreter','latex','fontsize',font_size)
hold off
axis tight
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.655, 0.155, 0.2, 0.2], 'EdgeColor', 'none');

%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================

%% Plot thrust vs time PID Aware of payload
set(figure,'Color','white','WindowState','maximized')
plot(PID.PayAwa.data.time,sum(PID.PayAwa.data.thrust,2),'b-','LineWidth',2)
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Total thrust [N]','interpreter','latex','fontsize',font_size)
title('PID','interpreter','latex','fontsize',font_size_title)
hold off
axis tight
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
legend('$u_1(t)$','Motor failure'); 
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.755, 0.2, 0.2], 'EdgeColor', 'none');




% axis ([min(PID.PayAwa.data.time), max(PID.PayAwa.data.time), min(sum(PID.PayAwa.data.thrust,2)),max(sum(PID.PayAwa.data.thrust,2))])

set(figure,'Color','white','WindowState','maximized')
subplot(2,1,1)
plot(PID.PayAwa.data.time,PID.PayAwa.data.thrust(:,1),'b-','LineWidth',2)
hold on
plot(PID.PayAwa.data.time,PID.PayAwa.data.thrust(:,2),'r-.','LineWidth',2)
plot(PID.PayAwa.data.time,PID.PayAwa.data.thrust(:,3),'g--','LineWidth',2)
plot(PID.PayAwa.data.time,PID.PayAwa.data.thrust(:,4),'k-','LineWidth',0.5)
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
legend('$T_1(t)$','$T_2(t)$','$T_3(t)$','$T_4(t)$','');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Thrust motor $i$ [N]','interpreter','latex','fontsize',font_size)
title('PID','interpreter','latex','fontsize',font_size_title)
hold off
axis tight
str={'Motor failure'};
annotation('textbox','interpreter','latex','String',str,'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.755, 0.2, 0.2], 'EdgeColor', 'none');

subplot(2,1,2)
plot(PID.PayAwa.data.time,PID.PayAwa.data.thrust(:,5),'b-','LineWidth',2)
hold on
plot(PID.PayAwa.data.time,PID.PayAwa.data.thrust(:,6),'r-.','LineWidth',2)
plot(PID.PayAwa.data.time,PID.PayAwa.data.thrust(:,7),'g--','LineWidth',2)
plot(PID.PayAwa.data.time,PID.PayAwa.data.thrust(:,8),'k-','LineWidth',0.5)
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
legend('$T_5(t)$','$T_6(t)$','$T_7(t)$','$T_8(t)$','Motor failure');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Thrust motor $i$ [N]','interpreter','latex','fontsize',font_size)
hold off
axis tight
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.28, 0.2, 0.2], 'EdgeColor', 'none');

%--------------------------------------------------------------------------
%% Plot angles vs. time PID Aware of payload
set(figure,'Color','white','WindowState','maximized')
subplot(3,1,1)
plot(PID.PayAwa.data.time,rad2deg(PID.PayAwa.data.roll),'b-','LineWidth',2)
hold on
plot(PID.PayAwa.data.time,rad2deg(PID.PayAwa.data.roll_ref),'r-.','LineWidth',2)
legend('$$\phi(t)$$','$$\phi_{\rm ref}(t)$$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Roll [deg]','interpreter','latex','fontsize',font_size)
title('PID','interpreter','latex','fontsize',font_size_title)
hold off
axis tight
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.755, 0.2, 0.2], 'EdgeColor', 'none');

subplot(3,1,2)
plot(PID.PayAwa.data.time,rad2deg(PID.PayAwa.data.pitch),'b-','LineWidth',2)
hold on
plot(PID.PayAwa.data.time,rad2deg(PID.PayAwa.data.pitch_ref),'r-.','LineWidth',2)
legend('$$\theta(t)$$','$$\theta_{\rm ref}(t)$$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Pitch [deg]','interpreter','latex','fontsize',font_size)
hold off
axis tight
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.455, 0.2, 0.2], 'EdgeColor', 'none');

subplot(3,1,3)
plot(PID.PayAwa.data.time,rad2deg(PID.PayAwa.data.yaw),'b-','LineWidth',2)
hold on
plot(PID.PayAwa.data.time,rad2deg(PID.PayAwa.data.yaw_ref),'r-.','LineWidth',2)
legend('$$\psi(t)$$','$$\psi_{\rm ref}(t)$$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Yaw [deg]','interpreter','latex','fontsize',font_size)
hold off
axis tight
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.155, 0.2, 0.2], 'EdgeColor', 'none');

%--------------------------------------------------------------------------
%% Plot position vs time PID Aware of payload

set(figure,'Color','white','WindowState','maximized')
subplot(3,1,1)
plot(PID.PayAwa.data.time,PID.PayAwa.data.translational_position_in_I(:,1),'r-','LineWidth',2)
hold on
plot(PID.PayAwa.data.time,PID.PayAwa.data.translational_position_in_I_user(:,1),'b-.','LineWidth',2)
legend('$r_x(t)$','$r_{\rm user,x}(t)$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('X position [m]','interpreter','latex','fontsize',font_size)
title('PID','interpreter','latex','fontsize',font_size_title)
hold off
axis tight
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.755, 0.2, 0.2], 'EdgeColor', 'none');


subplot(3,1,2)
plot(PID.PayAwa.data.time,PID.PayAwa.data.translational_position_in_I(:,2),'r-','LineWidth',2)
hold on
plot(PID.PayAwa.data.time,PID.PayAwa.data.translational_position_in_I_user(:,2),'b-.','LineWidth',2)
legend('$r_y(t)$','$r_{\rm user,y}(t)$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Y position [m]','interpreter','latex','fontsize',font_size)
hold off
axis tight
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.455, 0.2, 0.2], 'EdgeColor', 'none');

subplot(3,1,3)
plot(PID.PayAwa.data.time,-PID.PayAwa.data.translational_position_in_I(:,3),'r-','LineWidth',2)
hold on
plot(PID.PayAwa.data.time,-PID.PayAwa.data.translational_position_in_I_user(:,3),'b-.','LineWidth',2)
legend('$r_z(t)$','$r_{\rm user,z}(t)$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Z position [m]','interpreter','latex','fontsize',font_size)
hold off
axis tight
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.155, 0.2, 0.2], 'EdgeColor', 'none');

%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================


%% Plot Position error vs time ALL CONTROLLERS
set(figure,'Color','white','WindowState','maximized')
plot(PID.PayAwa.data.time,PID.PayAwa.pos_track_error_norm,'Color',"#0072BD",'LineStyle','-','LineWidth',2)
hold on
legend('PID');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('$\|x(t) - x_{\rm user}(t)\|$','interpreter','latex','fontsize',font_size)
title('Norm of trajectory tracking error','interpreter','latex','fontsize',font_size_title)
axis tight
hold off
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.755, 0.2, 0.2], 'EdgeColor', 'none');


