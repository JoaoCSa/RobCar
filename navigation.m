classdef navigation < matlab.System
    % Navigation block with ekf estimation

    % Public, tunable properties
    properties
        initial_x=0;                    % initial value of x
        initial_y=0;                    % initial value of y
        initial_theta=0;                % initial value of theta
        q_gain=0.001;
        r_gain=0.5;
    end

    properties(DiscreteState)
        
        est_vec;
        B_est;
        
        prev_x;
        prev_y;
        prev_theta;
        
        P;
        Q;
        R;
    end

    % Pre-computed constants
    properties(Access = private)
        L = 2.2;
        d = 0.64;
        r = 0.256;
        A = eye(3);
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants

            obj.prev_x=obj.initial_x;
            obj.prev_y=obj.initial_y;
            obj.prev_theta=obj.initial_theta;
            
            obj.est_vec = [0;0;0];
            obj.B_est = [cos(obj.initial_theta), 0; sin(obj.initial_theta), 0; 0, 1];
        
            obj.P = eye(3);
            obj.Q = obj.q_gain*eye(3);
            obj.R = obj.r_gain*[1 0 0
                                0 1 0
                                0 0 0.01];

        end

        function [x,y,theta] = stepImpl(obj,V,omega_s,x_real,y_real,GPS)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            
            u_vec = [V; omega_s];
            
            %% update omega_l, omega_r
            
            omega_l = -obj.d*omega_s/obj.r+V/obj.r + randn(1)*0.001;
            omega_r = 2*V/obj.r - omega_l + randn(1)*0.001;
            
            %% obtain measurements
            if GPS == 1
                % with GPS and wheel odometry
                [x_measured, y_measured, out1, out2] = gps_estimation(x_real, y_real, x_real, y_real);
                   
                theta_measured = obj.prev_theta + ((omega_r-omega_l)*obj.r)/(2*obj.d);        
            else
                % w/o GPS, kinematic model and odometry
                x_vec_meas = obj.A*obj.est_vec + obj.B_est*u_vec;
                x_measured = x_vec_meas(1);
                y_measured = x_vec_meas(2);
                
                theta_measured = obj.prev_theta + ((omega_r-omega_l)*obj.r)/(2*obj.d);
            end
            
            
            %% EKF estimation
            [x, y, theta, obj.P] = ekf_2w(obj.prev_x, obj.prev_y, obj.prev_theta, V, omega_s, obj.L, obj.P, x_measured, y_measured, theta_measured, obj.Q, obj.R);
    
            %% Estimated state and position variation
            obj.est_vec = [x; y; theta];
            obj.B_est = [cos(theta), 0; sin(theta), 0; 0, 1];
    
            %% update the prev values
            obj.prev_x = x;
            obj.prev_y = y;
            obj.prev_theta = theta;
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
    end
end
