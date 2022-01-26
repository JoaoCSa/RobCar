function u = feedback_vel(omegad,vd,err)
	Zeta = 0.9;
	g = 0.1;

	omega_n = sqrt(omegad^2 + g*vd^2);
	k1 = 2*Zeta*omega_n;
	k2 = g*abs(vd);
	k3 = k1;
	Ks = [k1,0,0;0,sign(vd)*k2,k3];
	
	uB = Ks*err;
	uF = [vd*cos(err(3));omegad];	
	u = uB + uF;
end