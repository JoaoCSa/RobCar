function [x,y,theta,phi, P] = ekf(x_prev,y_prev,theta_prev,phi_prev,V,omega_s, L, P, x_measured, y_measured, theta_measured, phi_measured, Q, R)

x_prev_vec = [x_prev; y_prev; theta_prev; phi_prev];
u_vec = [V; omega_s];
z = [x_measured; y_measured; theta_measured; phi_measured];

A = eye(4);
B = [cos(theta_prev), 0; sin(theta_prev), 0; (tan(phi_prev))/L, 0; 0, 1];

x_vec = A*x_prev_vec + B*u_vec;

F = [1 0 -V*sin(x_vec(3)) 0;
    0 1 V*cos(x_vec(3)) 0;
    0 0 1 (V/L)*(sec(x_vec(4))).^2;
    0 0 0 1];

H = eye(4);

y_tilde = z - x_vec;

P = F*P*transpose(F) + Q;

S = H*P*transpose(H) + R;

K = P*transpose(H)*inv(S);

x_est_vec = x_vec + K*y_tilde;

P = (eye(4)-K*H)*P;

% Q=eye(3);
% R=eye(3);
% P = [1^2 0 0 ; 0 1^2 0 ;0 0 (0.0001)^2];
% x_new = x_old + V*cos(theta_old);
% y_new = y_old + V*sin(theta_old);
% theta_new = theta_teo+ theta_teo*(10^(-5))*((rand(1,1) > 0.5)*2 - 1);
% F=[1 0 -V*sen(theta); 0 1 V*cos(theta); 0 0 1];
% P=F*P*F'+R;

x = x_est_vec(1);
y = x_est_vec(2);

if (abs(x_est_vec(3)) < 45)
    theta = x_est_vec(3);
else
    theta = theta_prev;
end

if (abs(x_est_vec(4)) < 45)
    phi = x_est_vec(4);
else
    phi = phi_prev;
end

end
