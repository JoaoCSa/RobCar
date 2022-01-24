clear;
%close all

x(1) = 0;
y(1) = 0;
theta(1) = 0;
phi(1) = 0;
L = 2.2;
V = 1;
omega_s = deg2rad(0.1);

A = eye(4);
B = [cos(theta(1)), 0; sin(theta(1)), 0; (tan(phi(1)))/L, 0; 0, 1];

P = [1 0 0 0
    0 1 0 0 
    0 0 1 0
    0 0 0 1];

Q = 0.01*[1 0 0 0
    0 1 0 0 
    0 0 0.1 0
    0 0 0 0.1];

R = 10*[1 0 0 0
    0 1 0 0 
    0 0 0.01 0
    0 0 0 0.001];

x_measured(1) = x(1);
y_measured(1) = y(1);

x_real(1) = x(1);
y_real(1) = y(1);
theta_real(1) = theta(1);
phi_real(1) = phi(1);

x_est(1) = x(1);
y_est(1) = y(1);
theta_est(1) = theta(1);
phi_est(1) = phi(1);

x_vec_real = [x_real; y_real; theta_real(1); phi_real(1)];

%% Estimation

state = [0;0;0;0]; %initial state

for i=1:300
    [x_measured(i), y_measured(i), x_real(i), y_real(i)] = gps_estimation(x_real(i), y_real(i), x_real(i), y_real(i));
    
    [x_var, y_var, theta_var, phi_var]=model(V,omega_s,theta_real(i),phi_real(i));
    
    %% Real
    x_real(i+1) =  x_real(i) + x_var;
    y_real(i+1) =  y_real(i) + y_var; 
    
    theta_real(i+1) = theta_real(i) + theta_var;
    
    if abs(phi_real(i) + phi_var) > deg2rad(45)
        phi_real(i+1) = phi_real(i);
    else 
        phi_real(i+1) = phi_real(i) + phi_var;
    end
    
    %% Measured / estimated
    x_measured(i+1) = x_measured(i) + x_var;
    y_measured(i+1) = y_measured(i) + y_var;
    
    theta_est(i+1) = theta_est(i) + theta_var;
       
    if abs(phi_est(i) + phi_var) > deg2rad(45)
        phi_est(i+1) = phi_est(i);
    else 
        phi_est(i+1) = phi_est(i) + phi_var;
    end
    %%
    state_meas = [x_measured(i); y_measured(i); theta_est(i); phi_est(i)];
    u = [V; omega_s]; %inputs [V,ws]
   
    %EKF = trackingEKF( @StateTransitionFcn, @MeasurementFcn ,state,'StateCovariance', 1, 'ProcessNoise',Q,'MeasurementNoise',R)
    EKF = trackingEKF( @StateTransitionFcn, @MeasurementFcn ,state, 'ProcessNoise',Q,'MeasurementNoise',R);
    %EKF = trackingEKF( @StateTransitionFcn, @MeasurementFcn ,state);
  
    % measurement = [x_real(i+1)+(-1) + (1-(-1))*rand(); y_real(i+1)+(-1) + (1-(-1))*rand();theta_real(i+1)+(-0.001) + (0.001-(-0.001))*rand();phi_real(i+1)]
    [state,P] = predict(EKF,u);
    [state, Pcorr] = correct(EKF,state_meas);
    %[state,P] = predict(EKF,u);
    
    x(i) = state(1);
    y(i) = state(2);
    theta(i) = state(3);
    phi(i) = state(4);
    
   
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


%% functions

function [state] = StateTransitionFcn(state, u)
    
    [x_var, y_var, theta_var, phi_var]=model(u(1),u(2),state(3),state(4));
    
%     state(1) = state(1) + x_var + (-0.01) + (0.01-(-0.01))*rand();
%     state(2) = state(2) + y_var + (-0.01) + (0.01-(-0.01))*rand();
    
    state(1) = state(1) + x_var;
    state(2) = state(2) + y_var;
    
    state(3) = state(3) + theta_var;
       
    if abs(state(4) + phi_var) > deg2rad(45)
        state(4) = state(4);
    else 
        state(4) = state(4) + phi_var;
    end
end

function [state] = MeasurementFcn(state)
    variation = 0;
    noise_x = (-variation) + (variation-(-variation))*rand();
    noise_y = (-variation) + (variation-(-variation))*rand();
    state(1) = state(1) + noise_x;
    state(2) = state(2) + noise_y;
    state(3) = state(3);
    state(4) = state(4);
end
