%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FALCO Drone Project 2022-23                                             %
% Quadrotor Parameters                                                    %
% Call to the Main Simulator                                              %
% Authors:  Pietro Bruschi                                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clearvars;
close all;
clc;

rng default

% Plot settings
set(0,'defaultTextInterpreter','latex');
set(0,'defaultFigurecolor','w');
set(0,'defaultLegendInterpreter','latex');
set(0,'defaultLineLineWidth',1)
set(groot,'DefaultAxesTickLabelInterpreter','latex');
set(groot,'DefaultAxesFontSize',22);

%% Physical Parameters 

m = 3.074;                                                                 % Drone Mass (nominal value) [kg]
J_xx_hat = 3.5; 
J_xx_unc = 20; 
J_yy_hat = 3.5; 
J_yy_unc = 20; 
J_zz_hat = 5; 
J_zz_unc = 20; 
J_xx_u = ureal('J_xx', J_xx_hat, 'Percentage', J_xx_unc);
J_yy_u = ureal('J_yy', J_yy_hat, 'Percentage', J_yy_unc);
J_zz_u = ureal('J_zz', J_zz_hat, 'Percentage', J_zz_unc);
b = 0.71;                                                                  % Arm Length 
sigma_hat = 1e-2;                                                          % Aerodynamics Coefficient
sigma_unc = 25;
sigma_u = ureal('sigma_u',sigma_hat,'Percentage',sigma_unc);

g = 9.81;                                                                  % Gravity Acceleration
g_vect = [0; 0; m*g];                                                      % Gravity Vector

% Adding an external force
f_x_hat = 0;
f_x_unc = m*g/50;
f_x_u = ureal('f_x_u',f_x_hat,'PlusMinus',f_x_unc);
f_y_hat = 0;
f_y_unc = m*g/50;
f_y_u = ureal('f_y_u',f_y_hat,'PlusMinus',f_y_unc);
f_z_hat = 0;
f_z_unc = m*g/50;
f_z_u = ureal('f_z_u',f_z_hat,'PlusMinus',f_z_unc);

tau = 1/25;                                                                % Actuator Delay [s/rad]

% Initial Conditions
p0 = zeros(3,1);                                                           % Initial position
v0 = zeros(3,1);                                                           % Initial velocity
q0 = [1 0 0 0]';                                                           % Initial attitude
omega0 = [0.01 0.01 0.01]';                                                % Initial angular rates

%% Controller Parameters

% Position Controller
Kp_x = 0.25;
Kp_y = 0.25;
Kp_z = 0.5;

% Velocity Controller
% (obtained by Simulink Tuning)

% Attitude Controller
Controllers.Kp_roll = 50;
Controllers.Kp_pitch = 50;
Controllers.Kp_yaw = 50;
Controllers.Kd_roll = 80;
Controllers.Kd_pitch = 80;
Controllers.Kd_yaw = 80;

%% Setpoint Generation

% For now done in SimuLink (3D step)

%% MC Simulation

n_MC = 50;
J_xx_MC = usample(J_xx_u,n_MC);
J_yy_MC = usample(J_yy_u,n_MC);
J_zz_MC = usample(J_zz_u,n_MC);

sigma_MC = usample(sigma_u,n_MC);

f_x_MC = usample(f_x_u,n_MC);
f_y_MC = usample(f_y_u,n_MC);
f_z_MC = usample(f_z_u,n_MC);

figure(1)
tlc = tiledlayout(2,2);

for i = 1:n_MC
    fprintf('Running Simulation n°%.0f \n',i)

    J_xx = squeeze(J_xx_MC(:,:,i));
    J_yy = squeeze(J_yy_MC(:,:,i));
    J_zz = squeeze(J_zz_MC(:,:,i));
    Drone.J = diag([ J_xx J_yy J_zz ]);                                    % Inertia Matrix of the drone [kg*m^2]
    Drone.Jinv = inv(Drone.J);                                             % Inverse Inertia Matrix

    sigma = squeeze(sigma_MC(:,:,i));
    mixer = [        -1         -1        -1         -1  ;                 % Mixer Matrix 
          -b/sqrt(2)  b/sqrt(2) b/sqrt(2) -b/sqrt(2) ;
           b/sqrt(2) -b/sqrt(2) b/sqrt(2) -b/sqrt(2) ;
              sigma      sigma    -sigma     -sigma  ];
    mixer_inv = inv(mixer);                                                % Inverse Mixer Matrix

    f_x = squeeze(f_x_MC(:,:,i));
    f_y = squeeze(f_y_MC(:,:,i));
    f_z = squeeze(f_z_MC(:,:,i));
    f_ext = [ f_x f_y f_z ]';

    sim_time = 30;
    simout = sim('MainDrone_fext','StopTime','sim_time'); 

    newcolors = eye(3);

    colororder(newcolors);

    nexttile(1)
    plot(simout.error);
    grid  minor
    xlabel('Time [s]', 'interpreter', 'latex')
    ylabel('State [m]', 'interpreter', 'latex')
    legend('x', 'y','z')
    hold on

    nexttile(2)
    plot(simout.position);
    grid minor
    xlabel('Time [s]', 'interpreter', 'latex')
    ylabel('Position [m]', 'interpreter', 'latex')
    legend('x', 'y','z')
    hold on

    nexttile(3)
    plot(simout.Forces);
    grid minor
    xlabel('Time [s]', 'interpreter', 'latex')
    ylabel('Force [N]', 'interpreter', 'latex')
    legend('$F_x$', '$F_y$','$F_z$')
    hold on 

    nexttile(4)
    plot(simout.Moments);
    grid minor
    xlabel('Time [s]', 'interpreter', 'latex')
    ylabel('Moments [Nm]', 'interpreter', 'latex')
    legend('$M_x$', '$M_y$','$M_z$')
    hold on
 
end

nexttile(1)
grid minor
nexttile(2)
grid minor
nexttile(3)
grid minor
nexttile(4)
grid minor

title(tlc, '\textbf{Monte Carlo Simulation}', 'interpreter', ...
    'latex', fontsize = 22)

%% Delete temporary files

if exist('slprj','dir')
    rmdir('slprj', 's')                                                    
end

%% END OF THE SCRIPT