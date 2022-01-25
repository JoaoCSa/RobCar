if exist('fig','var')
	for figk = fig
		clf(figk)
	end
end
clear;
cla;
traj = load('new_traj.mat');
N = length(traj.tSamples);

q = [0,0,0]; % [x,y,theta]
q_list = [];
omega_list = [];
T = traj.tSamples(2);
[OMEGAD, VD, THETAD] = estimate_state_accel(traj.qd(1,:),traj.qd(2,:),traj.qdd(1,:),traj.qdd(2,:),T);

THETAD = normalize_angle(THETAD);

i_init = 1;
% Estimate the trajectory
q_est_list = [];
q_est = [traj.q(1,i_init),traj.q(2,i_init),THETAD(i_init)];
for ii = i_init:N
    omegad = OMEGAD(ii);
    vd = VD(ii);
    q_est_list = [q_est_list;q_est];
    q_ref = [traj.q(:,ii).',THETAD(ii)];
    err = local_error(q_est,q_ref);
    u = feedback_vel(omegad,vd,err);
    q_est = q_est + kin_model(u(2),u(1),q_est(3),T);
end

fig = [];
fig = [fig,figure(1)];
hold on;
plot(traj.q(2,:),traj.q(1,:));
plot(q_est_list(:,2),q_est_list(:,1));
hold off;

% Dark plots
for figk = fig
	darkBackground(fig,[0.1 0.1 0.1],[0.5 0.7 0.7]);
	plot_darkmode;
end

function t = normalize_angle(angles)
	t = rem(abs(angles),2*pi);

    t(angles < 0) = -1*t(angles < 0);
    p = t;
    t(p < -pi) = t(p < -pi) + 2*pi;
    t(p > pi) = t(p > pi) - 2*pi;
end

function err = angle_error(phid,phie)
    %{
        Calculates the error between two angles.
        Input:
            phid: column vector
            phie: column vector
        Output:
            error
    %}
    p = phid;
    t = phid;
    p(p < 0) = p(p < 0) + 2*pi;
    t(t > 0) = t(t > 0) - 2*pi;
    error_p = p - phie;
    error_t = t - phie;
    
    err = zeros(size(phid));

    t_best = abs(error_p) > abs(error_t);
    p_best = logical(bitxor(t_best,1));

    err(t_best) = error_t(t_best);
    err(p_best) = error_p(p_best);
end

