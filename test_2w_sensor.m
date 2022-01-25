clear all;
close all;

x(1) = 0;
y(1) = 0; %5
theta(1) = 0;
%phi(1) = 0;
L = 2.2;
V = 1;
d = 0.64;
r = 0.256;


%omega_s(1) = deg2rad(0.005);
omega_s(1) = deg2rad(0.0);

omega_l(1) = -d*omega_s(1)/r+V/r;
omega_r(1) = 2*V/r - omega_l(1);

%theta_2(1) = ((omega_r(1)-omega_l(1))*r)/(2*d);
theta_2(1) = 0;

%% matrices

A = eye(3);
B = [cos(theta(1)), 0; sin(theta(1)), 0; 0, 1];

P = [1 0 0
    0 1 0 
    0 0 1];

Q = 0.001*[1 0 0
            0 1 0 
            0 0 1]; %0.001

R = 0.5*[1 0 0
        0 1 0
        0 0 0.01];

    
%% x,y,theta values    
x_measured(1) = x(1);
y_measured(1) = y(1);
theta_measured = theta(1);
%phi_measured = phi(1);

x_real(1) = x(1);
y_real(1) = y(1); %-5
theta_real(1) = theta(1);
%phi_real(1) = phi(1);

x_real_2(1) = x(1);
y_real_2(1) = y(1); %-5
theta_real_2 = theta_2(1);

x_vec_real = [x_real; y_real; theta(1)];
x_vec_real_2 = [x_real_2; y_real_2; theta_2(1)];

x_vec_est = [0;0;0];

x_var=150;
x_var_2=300;


% create simulation of 320 time steps
for i=1:400
    %% obtain measurements
    if i < 150 || i > 300
        [x_measured(i), y_measured(i), x_real(i), y_real(i)] = gps_estimation(x_real(i), y_real(i), x_real(i), y_real(i));
        %x_vec_meas = A*[x_real; y_real; theta(1); phi(1)] + [cos(x_vec_real(3)), 0; sin(x_vec_real(3)), 0; (tan(x_vec_real(4)))/L, 0; 0, 1]*u_vec;
        %x_vec_meas = A*x_vec_real + B_real*u_vec;
        
        %theta_measured(i) = x_vec_est(3)+ randn(1)*0.001;
        %theta_measured(i) = x_vec_est(3);
        %theta_measured(i+1) = x_vec_real(3)+ randn(1)*0.001;
        theta_measured(i+1) = x_vec_real_2(3)+ randn(1)*0.0001;
        
        %phi_measured(i+1) = x_vec_real(4)+ randn(1)*0.001;
    else
        x_vec_meas = A*est_vec + B_est*u_vec;
        %x_measured(i) = x_vec_meas(1)+ randn(1)*0.1;
        %y_measured(i) = x_vec_meas(2)+ randn(1)*0.1;
        %theta_measured(i) = x_vec_meas(3)+ randn(1)*0.01;
        
        x_measured(i) = x_vec_meas(1);
        y_measured(i) = x_vec_meas(2);
        %theta_measured(i) = x_vec_meas(3);
        %phi_measured(i+1) = x_vec_meas(4)+ randn(1)*0.001;
        
        theta_measured(i+1) = x_vec_real_2(3)+ randn(1)*0.0001;
    end
    %if (i > 50) && (i <= 100)
    %    x_measured(i+1) = x_measured(i+1) + (i - 50)*0.4;
    %    x_real(i+1) = x_real(i+1) + (i - 50)*0.4;
    %end
    
    %if (i > 100) && (i <= 150)
    %    x_measdured(i+1) = x_measured(i+1) + (50 - (i - 100))*0.4;
    %    x_real(i+1) = x_real(i+1) + (50 - (i - 100))*0.4;
    %end
    
    %% Real values
    B_real = [cos(x_vec_real(3)), 0; sin(x_vec_real(3)), 0; 0, 1];
    
    u_vec = [V; omega_s(i)];
    
    x_vec_real = A*x_vec_real + B_real*u_vec;
    x_real(i+1) = x_vec_real(1);
    y_real(i+1) = x_vec_real(2);
    theta_real(i+1) = x_vec_real(3);
    %phi_real(i+1) = x_vec_real(4);
    
    %% Real values to test w_l and w_r
    
    B_real_2 = [cos(theta_real_2(i)), 0; sin(theta_real_2(i)), 0; 0, 1];
    
    u_vec_2 = [V; ((omega_r(i)-omega_l(i))*r)/(2*d)];
    
    x_vec_real_2 = A*x_vec_real_2 + B_real_2*u_vec_2;
    x_real_2(i+1) = x_vec_real_2(1);
    y_real_2(i+1) = x_vec_real_2(2);
    theta_real_2(i+1) = x_vec_real_2(3);
    
    
    %phi_real(i+1) = x_vec_real(4);
    
    
    %% EKF estimation
    [x(i+1), y(i+1), theta(i+1), P] = ekf_2w(x(i), y(i), theta(i), V, omega_s(i), L, P, x_measured(i), y_measured(i), theta_measured(i), Q, R);
    %[x,y,theta,phi, P] = ekf(x_prev,y_prev,theta_prev,phi_prev,V,omega_s, L, P, x_measured, y_measured, theta_measured, phi_measured, Q, R)

    
    % To get the estimated value of theta (assuming it's not measured)
    est_vec = [x(i+1); y(i+1); theta(i+1)];
    B_est = [cos(theta(i+1)), 0; sin(theta(i+1)), 0; 0, 1];
    x_vec_est = A*est_vec + B_est*u_vec;
    
    
    %% update omega_s, omega_l, omega_r
    
    %omega_s(i+1) = omega_s(1)*cos((i*8)*pi/180);
    omega_s(i+1) = omega_s(i);
    %omega_s = cos(2*pi/(320/2)*i)*0.000;

    omega_l(i+1) = -d*omega_s(i+1)/r+V/r;
    omega_r(i+1) = 2*V/r - omega_l(i+1);

    if i==150
        x_var=x_real(i);
    elseif i==300
        x_var_2=x_real(i);
    end

end

figure
plot(x,y,'b')
hold on
plot(x_measured,y_measured,'c')
hold on
plot(x_real,y_real,'r')
xline(x_var, 'linewidth', 2);
xline(x_var_2, 'linewidth', 2);
legend("car kalman filtered global position","car measured global position","real position")
ylabel('y[m]')
xlabel('x[m]')
grid on

figure 
plot(theta,'b')
hold on
plot(theta_real,'.g')
plot(theta_measured, 'r')
grid on



figure
plot(x_real,y_real,'r')
hold on
plot(x_real_2,y_real_2,'.g')

% 
% figure 
% plot(phi,'b')
% hold on
% plot(phi_real,'.g')
% grid on