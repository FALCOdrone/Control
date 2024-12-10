%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FALCO Drone Project 2022-23                                             %
% Quadrotor Parameters                                                    %
% Call to the Main Simulator  
% Controller in C                                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clearvars;  
close all;
clear all;
clc;

% Plot settings
set(0,'defaultTextInterpreter','latex');
set(0,'defaultFigurecolor','w');
set(0,'defaultLegendInterpreter','latex');
set(0,'defaultLineLineWidth',1)
set(groot,'DefaultAxesTickLabelInterpreter','latex');
set(groot,'DefaultAxesFontSize',22);

%% Physical Parameters 

m = 3.074;                                                                 % Drone Mass [kg]
Drone.J = diag([ 2.5 2.5 5 ]);                                             % Inertia Matrix of the drone [kg*m^2]
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
%in controller_full.c
%% Setpoint Generation

% For now done in SimuLink (3D step)

% Update 21/07/23 
% Added command shaping filters to derive setpoint in velocity and
% acceleration.
% Reference: Handbook of Marine Craft Hydrodynamics and Motion Control, First Edition. Thor I. Fossen.
%            © 2011 John Wiley & Sons Ltd. Published 2011 by John Wiley & Sons Ltd. ISBN: 978-1-119-99149-6
%            ("command shaping filter" folder on OneDrive)


sim_time = 20;
dt=0.01;

% % Elica a spirale logaritmica
%  t=[0:0.1:sim_time];
%  r=exp(t/60);
%  x=r.*cos(t/20);
%  y=r.*sin(t/20);
%  z=-t;


% %elica
% t=[0:0.1:sim_time];
% x=sin(5*t);
% y=cos(5*t);
% z=t;

% %esponenziale
% t=[0:0.1:sim_time];
% z=1*(-1+exp(-t));
% y=0*t;
% x=-t;

%step
% t=[0:0.01:sim_time];
% z = 0*(t<=1) - 1*(t>1);
% y= 0*t;
% x= 0*t;

%3_step
t=0:0.01:sim_time;
z = 0*(t<=1) - 1*(t>1);
y= 0*(t<=5) - 1*(t>5);
x= 0*(t<=2) - 1*(t>2);

input_x = [t', x'];
input_y = [t', y'];
input_z = [t', z'];
figure('Name','Traiettoria');
plot3(x,y,z);

wn = 50;
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


simout = sim('DroneDynamics_full.slx','StopTime','sim_time'); 

% Position error
figure('name','ERROR');
plot(simout.error);
grid  minor
xlabel('Time [s]', 'interpreter', 'latex')
ylabel('State [m]', 'interpreter', 'latex')
legend('x', 'y','z')
title('\textbf{Evolution of the error dynamics}', ...
    'interpreter', 'latex', fontsize = 22)

% Dynamic Parameters Evolution
figure('name','DYNAMIC');
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
legend('roll','pitch','yaw')
title('')
nexttile
plot(simout.omega);
grid minor
xlabel('Time [s]', 'interpreter', 'latex')
ylabel('Angular Rates [rad/s]', 'interpreter', 'latex')
legend('roll_rate','pitch_rate','yaw_rate')
title('')

title(tlc2, '\textbf{Model Dynamics}', 'interpreter', ...
    'latex', fontsize = 22)

% Control Actions
figure('name','Forces/Moments actions');
nexttile
plot(simout.Forces);
grid minor
xlabel('Time [s]', 'interpreter', 'latex')
ylabel('Force [N]', 'interpreter', 'latex')
title('')
nexttile
plot(simout.Moments);
grid minor
xlabel('Time [s]', 'interpreter', 'latex')
ylabel('Moments [Nm]', 'interpreter', 'latex')
title('')

%Real action
figure('name','Forces/Moments real');
subplot(2,1,1);
title('')
plot(simout.M_real);
grid minor
xlabel('Time [s]', 'interpreter', 'latex')
ylabel('Moments [Nm]', 'interpreter', 'latex')
subplot(2,1,2);
title('')
plot(simout.F_real);
grid minor
xlabel('Time [s]', 'interpreter', 'latex')
ylabel('Force [N]', 'interpreter', 'latex')
title('')

% XYZ
figure('name','XYZ space')
plot3(simout.xd,simout.yd,simout.zd);
%% Delete temporary files

if exist('slprj','dir')
    rmdir('slprj', 's')                                                    
end

%% END OF THE SCRIPT