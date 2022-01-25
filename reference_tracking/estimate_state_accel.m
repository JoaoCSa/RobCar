function [OMEGA,V,THETA] = estimate_state(X_dot,Y_dot,X_dotdot,Y_dotdot,T)
	V_squared = X_dot.^2 + Y_dot.^2;

	THETA = atan2(Y_dot,X_dot);
	V = sqrt(V_squared);
	OMEGA = (X_dot.*Y_dotdot - Y_dot.*X_dotdot)./V_squared;
end