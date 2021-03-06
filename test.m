clear;
close all;

x(1) = 0;
y(1) = 0;
theta(1) = 0;
phi(1) = 0;
L = 2.2;
V = 1;
omega_s = deg2rad(0.01);

A = eye(4);
B = [cos(theta(1)), 0; sin(theta(1)), 0; (tan(phi(1)))/L, 0; 0, 1];

P = [1 0 0 0
    0 1 0 0 
    0 0 1 0
    0 0 0 1];

Q = 0.001*[1 0 0 0
    0 1 0 0 
    0 0 1 0
    0 0 0 1];

R = 10*[1 0 0 0
    0 1 0 0 
    0 0 1 0
    0 0 0 0.01];

x_measured(1) = x(1);
y_measured(1) = y(1);
theta_measured = theta(1);
phi_measured = phi(1);
x_real(1) = x(1);
y_real(1) = y(1);
theta_real(1) = theta(1);
phi_real(1) = phi(1);

x_vec_real = [x_real; y_real; theta(1); phi(1)];
x_vec_est = [0;0;0;0];

% create simulation of 320 time steps
for i=1:320
    % obtain measurements
    if i < 150
        [x_measured(i+1), y_measured(i+1), x_real(i+1), y_real(i+1)] = gps_estimation(x_real(i), y_real(i), x_real(i), y_real(i));
        %x_vec_meas = A*[x_real; y_real; theta(1); phi(1)] + [cos(x_vec_real(3)), 0; sin(x_vec_real(3)), 0; (tan(x_vec_real(4)))/L, 0; 0, 1]*u_vec;
        %x_vec_meas = A*x_vec_real + B_real*u_vec;
        
        theta_measured(i+1) = x_vec_est(3)+ randn(1)*0.001;
        %theta_measured(i+1) = x_vec_real(3)+ randn(1)*0.001;
        
        phi_measured(i+1) = x_vec_real(4)+ randn(1)*0.001;
    else
        x_vec_meas = A*x_vec_real + B_real*u_vec;
        x_measured(i+1) = x_vec_meas(1)+ randn(1)*0.1;
        y_measured(i+1) = x_vec_meas(2)+ randn(1)*0.1;
        theta_measured(i+1) = x_vec_meas(3)+ randn(1)*0.001;
        phi_measured(i+1) = x_vec_meas(4)+ randn(1)*0.001;
    end
    %if (i > 50) && (i <= 100)
    %    x_measured(i+1) = x_measured(i+1) + (i - 50)*0.4;
    %    x_real(i+1) = x_real(i+1) + (i - 50)*0.4;
    %end
    
    %if (i > 100) && (i <= 150)
    %    x_measdured(i+1) = x_measured(i+1) + (50 - (i - 100))*0.4;
    %    x_real(i+1) = x_real(i+1) + (50 - (i - 100))*0.4;
    %end
    
    B_real = [cos(x_vec_real(3)), 0; sin(x_vec_real(3)), 0; (tan(x_vec_real(4)))/L, 0; 0, 1];
    
    u_vec = [V; omega_s];
    
    x_vec_real = A*x_vec_real + B_real*u_vec;
    x_real(i+1) = x_vec_real(1);
    y_real(i+1) = x_vec_real(2);
    theta_real(i+1) = x_vec_real(3);
    phi_real(i+1) = x_vec_real(4);
    
    [x(i+1), y(i+1), theta(i+1), phi(i+1), P] = ekf(x(i), y(i), theta(i), phi(i), V, omega_s, L, P, x_measured(i+1), y_measured(i+1), theta_measured(i), phi_measured(i), Q, R);
    %[x,y,theta,phi, P] = ekf(x_prev,y_prev,theta_prev,phi_prev,V,omega_s, L, P, x_measured, y_measured, theta_measured, phi_measured, Q, R)

    
    % To get the estimated value of theta (assuming it's not measured)
    est_vec = [x(i+1); y(i+1); theta(i+1); phi(i+1)];
    B_est = [cos(theta(i+1)), 0; sin(theta(i+1)), 0; (tan(phi(i+1)))/L, 0; 0, 1];
    x_vec_est = A*est_vec + B_est*u_vec;
    
    
    
    %omega_s(i+1) = omega_s(1)*cos((i*8)*pi/180);
    %omega_s(i+1) = omega_s(i);
end

figure
plot(x,y,'b')
hold on
plot(x_measured,y_measured,'c')
hold on
plot(x_real,y_real,'r')
legend("car kalman filtered global position","car measured global position","real position")
ylabel('y[m]')
xlabel('x[m]')
grid on

figure 
plot(theta,'b')
hold on
plot(theta_real,'.g')
grid on

figure 
plot(phi,'b')
hold on
plot(phi_real,'.g')
grid on