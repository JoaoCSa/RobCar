function q_dot = kin_model(omega,v,theta,T)
	% Discrete time car kinematics
	% It assumes a constant linear velocity and angular velocity
	epsilon = 1e-6;
	theta_dot = omega*T;
	if abs(omega) > epsilon
		x_dot = v/omega*(sin(theta_dot + theta) - sin(theta));
		y_dot = - v/omega*(cos(theta_dot + theta) - cos(theta));
	else
		% If omega is really small
		x_dot = v*cos(theta)*T;
		y_dot = v*sin(theta)*T;
	end
	q_dot = [x_dot,y_dot,theta_dot];
end