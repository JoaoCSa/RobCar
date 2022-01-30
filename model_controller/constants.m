
clear out;
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
%kp = 50;
%ki = 100;
kp = 50;
ki = 100;
%ki = 50;
%kp = 100;
%ki = 0;
kd = 0;
N = 0;

trajectory = load('traj_maluca.mat');
T = trajectory.tSamples(2);
%{
trajectory.q = trajectory.q(:,50:end);
trajectory.qd = trajectory.qd(:,50:end);
trajectory.qdd = trajectory.qdd(:,50:end);
trajectory.tSamples = trajectory.tSamples(:,50:end);
%}
x_final = trajectory.q(1,end)
y_final = trajectory.q(2,end)


cont = kin_controller(trajectory);
[OMEGAD, VD, THETAD] = cont.estimate_state_accel(trajectory.qd(1,:),trajectory.qd(2,:),trajectory.qdd(1,:),trajectory.qdd(2,:));

%[OMEGAD, VD, THETAD] = cont.estimate_state_pos( ...
 %   trajectory.q(1,:),trajectory.q(2,:),trajectory.qd(1,:),trajectory.qd(2,:),T);

xdot =  trajectory.q(1,2) - trajectory.q(1,1);
ydot = trajectory.q(2,2) - trajectory.q(2,1);
theta_init = THETAD(1);
theta_init = atan2(ydot,xdot);
%theta_init = -pi/2;
M = [r/2,r/2;r/l,-r/l];
omegarl = M^(-1)*[VD;OMEGAD];
omega1 = OMEGAD(1:end-1);
omega2 = OMEGAD(2:end);

v1 = VD(1:end-1);
v2 = VD(2:end);
vdot = (v1-v2)/T;
omegadot = (omega1 - omega2)/T;

x_init = trajectory.q(1,1);
y_init = trajectory.q(2,1);


position_noise_mean = 2;
position_noise_variance = 0.5;
angular_speed_noise_mean = 0.001;
angular_speed_noise_variance = 0.5;
theta_noise_mean = 0.001;
theta_noise_variance = 0.001;

NoiseSampleTime = 0.00001;

seed1 = 1;
seed2 = 2;
seed3 = 3;
seed4 = 4;
seed5 = 5;
