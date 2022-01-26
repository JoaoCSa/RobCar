classdef navigation < matlab.System
    % Untitled Add summary here
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.

    % Public, tunable properties
    properties
        initial_x=0;                    % initial value of x
        initial_y=0;                    % initial value of y
        initial_theta=0;                % initial value of theta
        q_gain=0.001;
        r_gain=0.5;
    end

    properties(DiscreteState)
        %x_vec_real;             % initial state
        %x_vec_real_2;           % initial state
        
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
            %obj.A = eye(3);
            %obj.B = []
            obj.prev_x=obj.initial_x;
            obj.prev_y=obj.initial_y;
            obj.prev_theta=obj.initial_theta;
            
            %obj.x_vec_real_2 = [obj.initial_x; obj.initial_y; obj.initial_theta];
            
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
                
%                 obj.prev_x = obj.prev_x + 1;
%                 obj.prev_y = obj.prev_y + 1;
%                 obj.prev_theta = obj.prev_theta + 1;
%                 x = obj.prev_x;
%                 y = obj.prev_y;
%                 theta = obj.prev_theta;
%                 z = obj.prev_x;
%                 
            else
                % w/o GPS, kinematic model and odometry
                x_vec_meas = obj.A*obj.est_vec + obj.B_est*u_vec;
                x_measured = x_vec_meas(1);
                y_measured = x_vec_meas(2);
                
                %theta_measured = x_vec_real_2(3)+ randn(1)*0.001;
                
                theta_measured = obj.prev_theta + ((omega_r-omega_l)*obj.r)/(2*obj.d);
               
                
                %theta_measured(i) = x_vec_real_2(3);
            end
            
            %obj.x_vec_real = [x_real,y_real,V];
            %obj.x_vec_real_2 = [obj.x,obj.y,obj.theta];

            %% Real values
    
%             B_real = [cos(x_vec_real(3)), 0; sin(x_vec_real(3)), 0; 0, 1];
%             u_vec = [V; omega_s(i)];
% 
%             x_vec_real = obj.A*x_vec_real + B_real*u_vec;
%             x_real(i+1) = x_vec_real(1);
%             y_real(i+1) = x_vec_real(2);
%             theta_real(i+1) = x_vec_real(3);
%             %phi_real(i+1) = x_vec_real(4);

            %% Real values to test w_l and w_r

%             B_real_2 = [cos(theta_real_2(i)), 0; sin(theta_real_2(i)), 0; 0, 1];
%             u_vec_2 = [V; ((omega_r(i)-omega_l(i))*r)/(2*d)];
% 
%             obj.x_vec_real_2 = obj.A*obj.x_vec_real_2 + B_real_2*u_vec_2;
%             x_real_2(i+1) = x_vec_real_2(1);
%             y_real_2(i+1) = x_vec_real_2(2);
%             theta_real_2(i+1) = x_vec_real_2(3);
            
            %% EKF estimation
            [x, y, theta, obj.P] = ekf_2w(obj.prev_x, obj.prev_y, obj.prev_theta, V, omega_s, obj.L, obj.P, x_measured, y_measured, theta_measured, obj.Q, obj.R);
    
            %% Estimated state and position variation
            obj.est_vec = [x; y; theta];
            obj.B_est = [cos(theta), 0; sin(theta), 0; 0, 1];
    
            %% update the prev values
            obj.prev_x = x;
            obj.prev_y = y;
            obj.prev_theta = theta;
            %y = u;
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
    end
end
