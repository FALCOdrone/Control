%% Vehicle Nonlinear Variables
% This file was derived from the work by Peter Corke and Fabian Reither.
% Copyright (C) 1993-2015, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
%
% http://www.petercorke.com

% Copyright 2013-2023 The MathWorks, Inc.

% 6DOF
Vehicle.SixDOF.initGreenwich = 0;
Vehicle.SixDOF.quatGain = 1;
% Position on Earth
Vehicle.PositionOnEarth.href = -init.posLLA(3);
Vehicle.PositionOnEarth.FlatEarthToLLA.xAxis = 0;
%% Physical properties
% Airframe
switch (model)
    case 'Mambo'
        Vehicle.Airframe.mass = 3.76946;
        Vehicle.Airframe.inertia = diag([ 86445679.21e-9, 87526959.15e-9, 160140545.18e-9]);
        Vehicle.Airframe.d = 0.71/2;
        Vehicle.Rotor.radius = 0.2165;
        Vehicle.Rotor.bladeMass = 0.034;
        Vehicle.Airframe.h = -0.0685;
        Vehicle.Rotor.chord = .030;
        Vehicle.Rotor.theta0 = 30*(pi/180);
        Vehicle.Rotor.thetaTip = 10*(pi/180);
        Vehicle.Rotor.Ct = .0103;
        Vehicle.Rotor.Cq = .0009088;
    case 'RollingSpider'
        Vehicle.Airframe.mass = 0.068;
        Vehicle.Airframe.inertia = diag([0.0686e-3 0.092e-3 0.1366e-3]);
        Vehicle.Airframe.d = 0.0624;
        Vehicle.Rotor.radius = 0.033;
        Vehicle.Rotor.bladeMass = 0.0015/4;
        Vehicle.Airframe.h = -0.015876;
        Vehicle.Rotor.chord = .008;
        Vehicle.Rotor.theta0 = 14.6*(pi/180);
        Vehicle.Rotor.thetaTip = 6.8*(pi/180);
        Vehicle.Rotor.Ct = .0107;
        Vehicle.Rotor.Cq = Vehicle.Rotor.Ct*sqrt(Vehicle.Rotor.Ct/2);
end
Vehicle.Airframe.xy = Vehicle.Airframe.d*sqrt(2)/2; % For diamond rotor set-up
Vehicle.Airframe.Cdx = 0;
Vehicle.Airframe.Cdy = 0;
Vehicle.Airframe.diameter = .01; % For drag calculation purposes
% Rotor
Vehicle.Rotor.blades = 2;

Vehicle.Rotor.flappingOffset = 0;
Vehicle.Rotor.bladeInertia = Vehicle.Rotor.bladeMass*Vehicle.Rotor.radius^2/4;
Vehicle.Rotor.hubMass = 0;
Vehicle.Rotor.hubInertia = 0;
Vehicle.Rotor.inertia = Vehicle.Rotor.hubInertia + Vehicle.Rotor.bladeInertia;
Vehicle.Rotor.solidity = Vehicle.Rotor.chord*Vehicle.Rotor.blades/(pi*Vehicle.Rotor.radius);
Vehicle.Rotor.theta1 = Vehicle.Rotor.thetaTip-Vehicle.Rotor.theta0;
Vehicle.Rotor.theta34 = Vehicle.Rotor.theta0+0.75*Vehicle.Rotor.theta1;
Vehicle.Rotor.a = 5.5; % Lift slope
Vehicle.Rotor.area = pi*Vehicle.Rotor.radius^2;
Vehicle.Rotor.lock = rho*Vehicle.Rotor.a*Vehicle.Rotor.chord*Vehicle.Rotor.radius^4/...
    (Vehicle.Rotor.hubInertia+Vehicle.Rotor.bladeInertia);
Vehicle.Rotor.b = Vehicle.Rotor.Ct*rho*Vehicle.Rotor.area*Vehicle.Rotor.radius^2;
Vehicle.Rotor.k = Vehicle.Rotor.Cq*rho*Vehicle.Rotor.area*Vehicle.Rotor.radius^3;
Vehicle.Rotor.w2ToThrustGain = Vehicle.Rotor.Ct*rho*Vehicle.Rotor.area*Vehicle.Rotor.radius^2;
% Motors
Vehicle.Motor.maxLimit = 100;
Vehicle.Motor.minLimit = 1;
Vehicle.Motor.commandToW2Gain = 4100; %13840.8; % motor command for Rolling Spider (0-500) to motorspeed^2
Vehicle.Rotor.commandToThrust = Vehicle.Rotor.w2ToThrustGain*Vehicle.Motor.commandToW2Gain; % 0.4; %
disp(Vehicle.Rotor.commandToThrust*Vehicle.Motor.maxLimit);
Vehicle.Motor.thrustToMotorCommand = 1/Vehicle.Rotor.commandToThrust; % 1/(Vehicle.Rotor.w2ToThrustGain*Vehicle.Motor.commandToW2Gain); % 1/(0.043);