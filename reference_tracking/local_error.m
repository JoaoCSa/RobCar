function err = local_error(q,qd)
	R = @(theta) [cos(theta),sin(theta),0;-sin(theta),cos(theta),0;0,0,1];
	theta = q(3);
	ep = (qd - q)';
	err = R(theta)*ep;
end 