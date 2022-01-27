classdef navigation < matlab.System
    % Navigation block with ekf estimation

    % Public, tunable properties
    properties
        initial_x=0;                    % initial value of x
        initial_y=0;                    % initial value of y
        initial_theta=0;                % initial value of theta
        q_gain=0.001;
        r_gain=0.5;
        OffsetTime = 0.0 % Offset Time
        SampleTime = 0.1 % Sample Time
        position_noise = 0.01;
        orientation_noise = 0.01;
        angular_speed_noise = 0.01;
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

            sts = getSampleTime(obj);

        end

        function [x,y,theta] = stepImpl(obj,theta_measured,omega_r,omega_l,x_real,y_real,GPS)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            
            
            %% update omega_l, omega_r
            %omega_l = -obj.d*omega_s/obj.r+V/obj.r + randn(1)*b;
            %omega_r = 2*V/obj.r - omega_l + randn(1)*b;
            
            omega_l_noise = omega_l + randn(1)*obj.angular_speed_noise;
            omega_r_noise = omega_r + randn(1)*obj.angular_speed_noise;
            omega_s = ((omega_r_noise - omega_l_noise)*obj.r)/(2*obj.d);

            V = (omega_r_noise + omega_l_noise)*obj.r/2;
            u_vec = [V; omega_s];
            %% obtain measurements
            if GPS == 1
                % with GPS and wheel odometry
                [x_measured, y_measured, out1, out2] = obj.gps_estimation(x_real, y_real, x_real, y_real);
                
                theta_measured = theta_measured + obj.orientation_noise*randn(1);
                %theta_measured = obj.prev_theta + omega_s*obj.SampleTime;      
            else
                % w/o GPS, kinematic model and odometry
                x_vec_meas = obj.A*obj.est_vec + obj.B_est*u_vec;
                x_measured = x_vec_meas(1);
                y_measured = x_vec_meas(2);
                
                theta_measured = obj.prev_theta + omega_s*obj.SampleTime;
            end
            
            %% EKF estimation
            [x, y, theta, obj.P] = obj.ekf_2w(obj.prev_x, obj.prev_y, obj.prev_theta, V, omega_s, obj.L, obj.P, x_measured, y_measured, theta_measured, obj.Q, obj.R);
    
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

        function [x_est,y_est,x_real,y_real] = gps_estimation(obj,x,y,x_r,y_r)
            %noise_x = (-1) + (1-(-1))*rand();
            %noise_y = (-1) + (1-(-1))*rand();
    
            noise_x = randn(1)*obj.position_noise;
            noise_y = randn(1)*obj.position_noise;
            x_est = x + noise_x;
            y_est = y + noise_y;
            x_real = x_r;
            y_real = y_r;
        end

        function [x,y,theta,P] = ekf_2w(obj,x_prev,y_prev,theta_prev,V,omega_s, L, P, x_measured, y_measured, theta_measured, Q, R)
            x_prev_vec = [x_prev; y_prev; theta_prev];
            u_vec = [V; omega_s];
            z = [x_measured; y_measured; theta_measured];

            A = eye(3);
            B = [cos(theta_prev), 0; sin(theta_prev), 0; 0, 1];

            x_vec = A*x_prev_vec + B*u_vec;

            F = [1 0 -V*sin(x_vec(3));
                0 1 V*cos(x_vec(3));
                0 0 1];

            H = eye(3);

            y_tilde = z - x_vec;

            P = F*P*transpose(F) + Q;

            S = H*P*transpose(H) + R;

            K = P*transpose(H)*inv(S);

            x_est_vec = x_vec + K*y_tilde;

            P = (eye(3)-K*H)*P;

            x = x_est_vec(1);
            y = x_est_vec(2);


            theta = x_est_vec(3);
        end

        function sts = getSampleTimeImpl(obj)
            % Sets the sample time
            sts = createSampleTime(obj,'Type','Discrete', ...
                'SampleTime',obj.SampleTime, ...
                'OffsetTime',obj.OffsetTime);
        end

        function flag = isInactivePropertyImpl(obj,prop)
            % Makes everything inactive between each step
            flag = false;
    
            if any(strcmp(prop,{'TickTime'}))
                flag = true;
            end
        end

    end
end
