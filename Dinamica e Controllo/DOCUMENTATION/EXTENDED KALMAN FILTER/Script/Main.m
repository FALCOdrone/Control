%%
% EKF
% Mechatronic systems and Lab. 
% Example N°1
% Riva Emanuele ** emanuele.riva@polimi.it **

clear 
close all
clc

% Data of the pendulum
g = 9.81; % [m/s^2] gravity
L = 0.1; % [m] length of the pendulum
m = 1; % [kg] mass of the pendulum
w0 = sqrt(g/L);
zeta = 0.02;

load('Optimal_Trajectory.mat')

X = Optimal_Trajectory.X; X = X(:,2:3);
c = Optimal_Trajectory.c; c = [c(:,:),c(:,end)];
Tc = Optimal_Trajectory.Tc;
Tx = Optimal_Trajectory.Tx;

% weight on the control action
R = 1;

% weight on the state
Q = zeros(2);
Q(1,1) = 1;
Q(2,2) = 1;

% weight on the final target
P = zeros(2);
P(1,1) = 0;
P(2,2) = 0;

% Normalization variables
dQpMax = 1000;          %[rad/s]
dQMax = 1;              %[rad]
dCMax = 1;              %[Nm]

Q(1,1) = Q(1,1)./dQpMax.^2;
Q(2,2) = Q(2,2)./dQMax.^2;

P(1,1) = P(1,1)./dQpMax.^2;
P(2,2) = P(2,2)./dQMax.^2;

R = R./dCMax.^2;

% Time discretization in N = number of time intervals
N = 1000;

Ns = size(X,2); % number of states
Nc = size(c,1); % number of controls

% Sampled time history
T = linspace(0,Tx(end),N);

% Interpolation/sampling of the state and control 
xk = zeros(N,Ns);
ck = zeros(N,Nc);
for ii = 1:Ns
    xk(:,ii) = interp1(Tx,X(:,ii),T);
end
for ii = 1:Nc
    ck(:,ii) = interp1(Tc,c(ii,:),T);
end

%------ INTEGRATION OF THE DRE EQUATION FOR OPTIMAL CONTROL PROBLEM ------%

% Option for ODE solver
VectTol = ones(Ns^2,1)*1e-5;
options = odeset('RelTol', 1e-5, 'AbsTol',VectTol);

% initial and final time
t0 = T(1);
Tf = T(end);

% boundary conditions
p0 = P(1:end)';

% Integration of the matrix riccati equation
[Tp,PP] = ode45(@(t,p) DRE(t,p,Q,R,X,Tx,c,Tc,w0,L,zeta,m), flip(T), p0, options);

% From backward to forward dynamics
PP = flipud(PP);

% Transformation Vector -> Matrix
PP_Matrix = zeros(Ns,Ns);

% Computation of the gain matrix, Uncontrolled stability matrix,
% Controlled stability matrix along the trajectory
K = zeros(N,Nc,Ns);
A = zeros(N,Ns,Ns);
Fc = zeros(N,Ns,Ns);
PolesUC = zeros(N,Ns);
PolesC = zeros(N,Ns);
for ii = 1:N
    % transformation vector -> matrix
    PP_Matrix(1:end) = PP(ii,:)';
    % control matrix G
    B = fu(xk(ii,:),ck(ii,:),w0,L,zeta,m);
    % Uncontrolled state stability matrix
    A(ii,:,:) = fx(xk(ii,:),ck(ii,:),w0,L,zeta,m);
    % Gain matrix C
    K(ii,:,:) = R^-1*B'*PP_Matrix; 
end

T = T';

%% EXTENDED KALMAN FILTER

% weight matrices
Qk = 1;
Rk = 1;

% observation matrix
C = [0,1];

%% Simulink Model

IC = [0;0];
IC_kf = zeros(2,1);
C1 = squeeze(K(:,1,:));
Wamp = 0*5e-3;
Wfreq = 10;      %[Hz]
Namp = 1*10^-1;

sim('Sim_exe.slx')

Xs = squeeze(Xs.data);
Xobs = squeeze(Xobs.data);
U = squeeze(U.data);
Uopt = squeeze(Uopt.data);
time = squeeze(time.data);
Xref = squeeze(Xref.data);

% observed quantities + control action
FigTag = figure;
ax = axes;
plot(time,Xobs(:,1),'LineWidth',1,'LineStyle','-','color','b');
hold on; grid on; box on;
plot(time,Xobs(:,2),'LineWidth',1,'LineStyle','-','color','r');
xlabel('t [s]','Interpreter','LaTex')
ylabel('$\dot\theta,\;\theta$','Interpreter','LaTex')
ax.FontSize = 16;
ax.TickLabelInterpreter = 'LaTex';
legend({'$\dot\theta$','$\theta$'},'Interpreter','LaTex','Location','northeast')


% print(FigTag,'fig41.jpeg','-djpeg','-r600')

%___ Observed vs measured quantities _____________________________________%

FigTag = figure;
ax = axes;
h = plot(time,Xs(:,1),'LineWidth',1,'color','b'); hold on; grid on; box on;
h.Color = [0,0,1,0.3];
plot(time,Xobs(:,2),'LineWidth',1,'LineStyle','--','color','b');
xlabel('t','Interpreter','LaTex')
ylabel('$\theta$','Interpreter','LaTex')
legend({'Measured','Observed'},'Interpreter','LaTex','Location','northeast')
ax.FontSize = 16;
ax.TickLabelInterpreter = 'LaTex';
drawnow

% print(FigTag,'fig51.jpeg','-djpeg','-r600')


% neighboring observed quantities + control action
FigTag = figure;
ax = axes;
h = scatter(Xobs(:,2),Xobs(:,1),1,time);
grid on; box on; 
colormap jet;
xlabel('$\theta$','Interpreter','LaTex')
ylabel('$\dot\theta$','Interpreter','LaTex')
ax.FontSize = 16;
ax.TickLabelInterpreter = 'LaTex';
hold on; 


