%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FALCO Drone Project 2022-23                                             %
% Quadrotor Parameters                                                    %
% Call to the Main Simulator                                              %
% Authors:  Pietro Bruschi                                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 28-02-2024 Update attitude gains, according to new saturation values

clearvars;  
close all;
clc;

% Plot settings:
set(0,'defaultTextInterpreter','latex');
set(0,'defaultFigurecolor','w');
set(0,'defaultLegendInterpreter','latex');
set(0,'defaultLineLineWidth',1)
set(groot,'DefaultAxesTickLabelInterpreter','latex');
set(groot,'DefaultAxesFontSize',22);

%% Physical Parameters 

m = 3.76946;                                                               % Drone Mass [kg]
Drone.J = diag([ 86445679.21e-9, 87526959.15e-9, 160140545.18e-9]);        % Inertia Matrix of the drone [kg*m^2]
Drone.Jinv = inv(Drone.J);                                                 % Inverse Inertia Matrix
b = 0.71;                                                                  % Arm Length 
sigma = 1e-2;                                                              % Aerodynamics Coefficient

g = 9.81;                                                                  % Gravity Acceleration
g_vect = [0; 0; m*g];                                                      % Gravity Vector

mixer = [        -1         -1        -1         -1  ;                     % Mixer Matrix 
          -b/sqrt(2)  b/sqrt(2) b/sqrt(2) -b/sqrt(2) ;
           b/sqrt(2) -b/sqrt(2) b/sqrt(2) -b/sqrt(2) ;
              sigma      sigma    -sigma     -sigma  ];

mixer_inv = inv(mixer);                                                    % Inverse Mixer Matrix

% tau = 1/25;                                                                % Actuator Delay [s/rad]
tau = 1/900;                                                                % Actuator Delay [s/rad]

% Initial Conditions
p0 = zeros(3,1);                                                           % Initial position
v0 = zeros(3,1);                                                           % Initial velocity
q0 = [1 0 0 0]';                                                           % Initial attitude
% omega0 = [0.01 0.01 0.01]';                                                % Initial angular rates
omega0 = zeros(3,1);                                                       % Initial angular rates

%% Controller Parameters

% Position Controller
Kp_x = 0.5;
Kp_y = 0.5;
Kp_z = 1;

% Velocity Controller

Kp_vx = 5;
Kp_vy = 5;
Kp_vz = 15;

Ki_vx = 0.5;
Ki_vy = 0.5;
Ki_vz = 10;

Kd_vx = 0.5;
Kd_vy = 0.5;
Kd_vz = 1;

% Attitude Controller
Controllers.Kp_roll = 5;
Controllers.Kp_pitch = 5;
Controllers.Kp_yaw = 5;
Controllers.Kd_roll = 1;
Controllers.Kd_pitch = 1;
Controllers.Kd_yaw = 1;

%% Setpoint Generation

% For now done in SimuLink (3D step)

% Update 21/07/23 
% Added command shaping filters to derive setpoint in velocity and
% acceleration.
% Reference: Handbook of Marine Craft Hydrodynamics and Motion Control, First Edition. Thor I. Fossen.
%            © 2011 John Wiley & Sons Ltd. Published 2011 by John Wiley & Sons Ltd. ISBN: 978-1-119-99149-6
%            ("command shaping filter" folder on OneDrive)

wn = 5;
xi = 0.9;

Omega = wn*eye(3);
Delta = xi*eye(3);

guidance.A_d = [  zeros(3)   eye(3)                     zeros(3)              ;
                  zeros(3)   zeros(3)                   eye(3)                ;
                 -Omega^3  -(2*Delta+eye(3))*Omega^2  -(2*Delta+eye(3))*Omega ];

guidance.B_d = [ zeros(3) ;
                 zeros(3) ;
                 Omega^3  ];

%% Animation

% Rotate from NED to ECI
NED2ECI = [ 1  0  0 ; 
            0 -1  0 ; 
            0  0 -1 ];

%% Simulation

% Simulate the closed-loop dynamics 

sim_time = 30;
simout = sim('DroneDynamics.slx','StopTime','sim_time'); 

% Position error
figure(1)
plot(simout.error);
grid  minor
xlabel('Time [s]', 'interpreter', 'latex')
ylabel('State [m]', 'interpreter', 'latex')
legend('x', 'y','z')
title('\textbf{Evolution of the error dynamics}', ...
    'interpreter', 'latex', fontsize = 22)

% Dynamic Parameters Evolution
figure(2)
tlc2 = tiledlayout(2,2);
nexttile
plot(simout.position);
grid minor
xlabel('Time [s]', 'interpreter', 'latex')
ylabel('Position [m]', 'interpreter', 'latex')
legend('x','y','z');
title('')
nexttile
plot(simout.velocity);
grid minor
xlabel('Time [s]', 'interpreter', 'latex')
ylabel('Velocity [m/s]', 'interpreter', 'latex')
legend('x','y','z');
title('')
nexttile
plot(simout.quaternions);
grid minor
xlabel('Time [s]', 'interpreter', 'latex')
ylabel('Attitude [rad]', 'interpreter', 'latex')
legend('x','y','z');
title('')
nexttile
plot(simout.omega);
grid minor
xlabel('Time [s]', 'interpreter', 'latex')
ylabel('Angular Rates [rad/s]', 'interpreter', 'latex')
legend('x','y','z');
title('')

title(tlc2, '\textbf{Model Dynamics}', 'interpreter', ...
    'latex', fontsize = 22)

% Control Actions
figure(3)
tlc3 = tiledlayout(2,1);
nexttile
plot(simout.Forces);
grid minor
xlabel('Time [s]', 'interpreter', 'latex')
ylabel('Force [N]', 'interpreter', 'latex')
legend('x','y','z');
title('')
nexttile
plot(simout.Moments);
grid minor
xlabel('Time [s]', 'interpreter', 'latex')
ylabel('Moments [Nm]', 'interpreter', 'latex')
legend('x','y','z');
title('')

%% Delete temporary files

if exist('slprj','dir')
    rmdir('slprj', 's')                                                    
end

%% END OF THE SCRIPT