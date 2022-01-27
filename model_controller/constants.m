% ENUNCIADO
m = 810; % massa
r = 0.256; % raio
d = 1.1 ; % centro massa 
L2 = 2.2;
l = 0.64;

% Actuator Parameters
L = 0.125; % inductance
R = 0.688; % resistance

Kt = 11.43; % torque constant
Kb = 0.06; % back emf constant

J = 0.0025; % momento de inercia

% PID 
kp = 100; 
ki = 50;
kd = 0;
N = 18.3;

trajectory = load('new_traj.mat');
T = trajectory.tSamples(2);


cont = kin_controller(trajectory);
[OMEGAD, VD, THETAD] = cont.estimate_state_accel(trajectory.qd(1,:),trajectory.qd(2,:),trajectory.qdd(1,:),trajectory.qdd(2,:),T);

xdot = trajectory.q(1,1) - trajectory.q(1,end);
ydot = trajectory.q(2,1) - trajectory.q(2,end);
theta_init = THETAD(1);
%theta_init = atan2(ydot,xdot);
%theta_init = pi;
M = [r/2,r/2;r/l,-r/l];
omegarl = M^(-1)*[VD;OMEGAD];
omega1 = OMEGAD(1:end-1);
omega2 =OMEGAD(2:end);

v1 = VD(1:end-1);
v2 = VD(2:end);
vdot = (v1-v2)/T;
omegadot = (omega1 - omega2)/T;

x_init = trajectory.q(1,1);
y_init = trajectory.q(2,1) - 10;
