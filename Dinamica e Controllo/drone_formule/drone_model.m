clc
clear all 
close all

syms x y z real
syms vx vy vz real
syms ax ay az real
syms phi theta psi real %euler angles
syms phidot thetadot psidot real
syms phiddot thetaddot psiddot real 
syms m g Ixx Iyy Izz real 
syms K l b Ax Ay Az real
syms omega1 omega2 omega3 omega4 real
syms p1 q1 r1 real
syms pp1 qq1 rr1 real
syms m1 m2 m3 real

i = [1 0 0]';
j = [0 1 0]';
k = [0 0 1]';

%%%%%%%%%% position %%%%%%%%%%
R_x = [1    0       0; ...
       0  cos(phi) -sin(phi); ...
       0  sin(phi) cos(phi)];
   
R_y = [cos(theta)  0   sin(theta); ...
       0           1         0; ...
      -sin(theta) 0   cos(theta)]; 
   
R_z = [cos(psi) -sin(psi)  0; ...
       sin(psi)  cos(psi)  0; ...
       0           0       1]; 


%%%% follow 3-2-1, rotate about z then y then x %%%%%
%%% We want to find R_01 such that: r_0 = R_01 r_1 
% r_0 = Rz r_z 
% r_z = Ry r_y
% r_y = Rx r_1
% Thus, r_0 = Rz Ry Rz r_1 = R_01 r_1
R = R_z*R_y*R_x;


% Similarly, r_1 = R_01^(-1) r_0 = R_01^T r_0
% But R_01^T = R_10 = (Rz Ry Rx)^T = Rx^T Ry^T Rz^T 
   
%%%%% angular velocity and energy %%%%
om_b = p1*i +  q1*j + r1*k;
I = [Ixx 0 0; 0 Iyy 0; 0 0 Izz];%body frame inertia
v = [vx; vy; vz];
T =  0.5*m*(v')*v + 0.5*om_b'*I*om_b;
V = m*g*z;
L = T-V;
% disp('copy paste energy in the code');
% disp(['KE(i) = ',char(T),';']);
% disp(['PE(i) = ',char(V),';']);
% disp(['TE(i) = KE(i)+PE(i);']);
% disp(' ');

%Derive equations of motion using Euler-Lagrange method
q = [x y z  phi theta psi ];
qdot = [vx vy vz p1 q1 r1];
qddot = [ax ay az pp1 qq1 rr1];


%%%%%% external forces and torques %%%%%%%%%
Thrust = [0; 0; K*(omega1^2+omega2^2+omega3^2+omega4^2)];
% Drag = [Ax*vx; Ay*vy; Az*vz];
F_ext =R*Thrust; % R*Thrust-Drag;
tau_phi = K*l*(omega4^2 - omega2^2)-p1*r1*(Izz-Iyy); %ultima sottrazione dipende dal cambio di cordinate
tau_theta = K*l*(omega3^2 - omega1^2)-p1*r1*(Ixx-Izz);
tau_psi = b*(omega1^2-omega2^2+omega3^2-omega4^2)-p1*q1*(Iyy-Ixx);
tau_ext = [tau_phi; tau_theta; tau_psi];




T_ext = [F_ext; tau_ext];



for ii=1:6
    dLdqdot(ii) = diff(L,qdot(ii));
    ddt_dLdqdot(ii) = diff(dLdqdot(ii),q(1))*qdot(1) + diff(dLdqdot(ii),qdot(1))*qddot(1)+...
                      diff(dLdqdot(ii),q(2))*qdot(2) + diff(dLdqdot(ii),qdot(2))*qddot(2)+...
                      diff(dLdqdot(ii),q(3))*qdot(3) + diff(dLdqdot(ii),qdot(3))*qddot(3)+...
                      diff(dLdqdot(ii),q(4))*qdot(4) + diff(dLdqdot(ii),qdot(4))*qddot(4)+...
                      diff(dLdqdot(ii),q(5))*qdot(5) + diff(dLdqdot(ii),qdot(5))*qddot(5)+...
                      diff(dLdqdot(ii),q(6))*qdot(6) + diff(dLdqdot(ii),qdot(6))*qddot(6);
    dLdq(ii) = diff(L,q(ii));

    EOM(ii) = ddt_dLdqdot(ii) - dLdq(ii) - T_ext(ii);
end


%%%%%%% post process equations %%%%%%%
A = jacobian(EOM,qddot);
for ii=1:6
    B(ii,1) = -subs(EOM(ii),qddot,[0 0 0 0 0 0]);
end

disp('copy paste in MATLAB');
disp(' ');
for ii=1:6
    for jj=1:6
        string = [ 'A(',num2str(ii),',',num2str(jj),')='];
        disp([string, char(simplify(A(ii,jj))), ';']);
    end
end

disp(' ');
for ii=1:6
    string = [ 'B(',num2str(ii),',1)='];
    disp([string, char(simplify(B(ii,1))), ';']);
end

disp(' ');
disp('X = A\B;');
disp(' ');
disp(A\B);

params.m = 0.680;
params.g = 9.81;
params.Ixx = 0.17414275;
params.Iyy = 0.17414275;
params.Izz = 0.17684675;
params.l = 0.225;
params.K = 2.980*1e-6;
params.b = 1.14*1e-7;


omega = 1.20; % da trovare 
speed = omega*sqrt(1/params.K);
dspeed = 0.05*speed;
params.omega1 = speed;
params.omega2 = speed;
params.omega3 = speed;
params.omega4 = speed;


F = A\B;
F = subs(F ,m , params.m);
F = subs(F ,Ixx , params.Ixx);
F = subs(F ,Iyy , params.Iyy);
F = subs(F ,Izz , params.Izz);
F = subs(F ,g , params.g);
F = subs(F ,K , params.K);
F = subs(F ,l , params.l);
F = subs(F ,b , params.b);
F = subs(F ,omega1 , params.omega1);
F = subs(F ,omega2, params.omega2);
F = subs(F ,omega3, params.omega3);
F = subs(F ,omega4, params.omega4);

%% ingressi del sitema per determinare le velocità
F = subs(F ,phi , pi/8);
F = subs(F ,theta , pi/4);
F = subs(F ,psi , pi/3);
F = subs(F ,p1 , 20);
F = subs(F ,q1 , 1);
F = subs(F ,r1 , 10);



F= vpa(F,7);

disp(F);