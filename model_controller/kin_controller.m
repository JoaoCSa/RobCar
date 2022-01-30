classdef kin_controller < matlab.System
    % Kinematic controller

    % Public, tunable properties
    properties
        traj = struct
        OffsetTime = 0.0 % Offset Time
        SampleTime = 0.1 % Sample Time
        useKinematicModel = false;
        useFeedback = true;
    end

    properties(DiscreteState)
        ii
        OMEGAD
        VD
        THETAD
        N
        T
        q_est
    end

    % Pre-computed constants
    properties(Access = private)

    end

    methods
        % Constructor method
        function obj = kin_controller(trajectory)
            % Saves the trajectory
            if nargin > 0
                obj.traj = trajectory
            end
        end

        function [OMEGA,V,THETA] = estimate_state_accel(obj,X_dot,Y_dot,X_dotdot,Y_dotdot)
            % Given the velocities and accelerations in the XY plane it estimates the angular 
            % and linear velocities of the car
            epsilon = 0.001;

            V_squared = X_dot.^2 + Y_dot.^2;

            V = sqrt(V_squared);
            OMEGA = (X_dot.*Y_dotdot - Y_dot.*X_dotdot)./V_squared;
            OMEGA(V < epsilon) = 0;

            THETA = zeros(size(X_dot));
            for ii = (2:length(X_dot))
                THETA(ii) = atan2(Y_dot(ii),X_dot(ii));
                if V(ii) < epsilon
                    THETA(ii) = THETA(ii - 1);
                end
            end

            for ii = (length(X_dot) - 1:-1:1)
                THETA(ii) = atan2(Y_dot(ii),X_dot(ii));
                if V(ii) < epsilon
                    THETA(ii) = THETA(ii + 1);
                end
            end

            %THETA(V < epsilon) = 0;

        end

        function [OMEGA,V,THETA] = estimate_state_pos(obj,X,Y,X_dot,Y_dot,T)
            % Given the velocities and accelerations in the XY plane it estimates the angular 
            % and linear velocities of the car
            jj = 1;
            kk = 1;

            X1 = X(1:end - jj);
            X2 = X(jj + 1:end);

            Y1 = Y(1:end - jj);
            Y2 = Y(jj + 1:end);
            
            THETA = atan2(Y2 - Y1,X2 - X1);

            THETA1 = THETA(1:end - kk);
            THETA2 = THETA(kk + 1:end);

            OMEGA = (THETA2 - THETA1)/(kk*T);


            V_squared = X_dot.^2 + Y_dot.^2;
            V_squared = V_squared(1:length(OMEGA));

            V = sqrt(V_squared);
            V = V(1:length(OMEGA));
        end

        function q_dot = kin_model(obj,omega,v,theta,T)
            % Discrete time car kinematics
            % It assumes a constant linear velocity and angular velocity
            % Given the linear and angular velocity computes it determines
            % the change in velocity in the XY plane and the change in theta
            epsilon = 1e-6;
            theta_dot = omega*T;
            if abs(omega) > epsilon
                x_dot = v/omega*(sin(theta_dot + theta) - sin(theta));
                y_dot = - v/omega*(cos(theta_dot + theta) - cos(theta));
            else
                % If omega is really small
                x_dot = v*cos(theta)*T;
                y_dot = v*sin(theta)*T;
            end
            q_dot = [x_dot,y_dot,theta_dot];
        end

        function t = normalize_angle(obj,angles)
            % Ensures that the angles are between -pi and pi
            t = rem(abs(angles),2*pi);

            t(angles < 0) = -1*t(angles < 0);
            p = t;
            t(p < -pi) = t(p < -pi) + 2*pi;
            t(p > pi) = t(p > pi) - 2*pi;
        end

        function err = angle_error(obj,phid,phie)
                %{
                Calculates the error between two angles.
                Input:
                    phid: column vector
                    phie: column vector
                Output:
                    error
                %}
            p = phid;
            t = phid;
            p(p < 0) = p(p < 0) + 2*pi;
            t(t > 0) = t(t > 0) - 2*pi;
            error_p = p - phie;
            error_t = t - phie;
            
            err = zeros(size(phid));

            t_best = abs(error_p) > abs(error_t);
            p_best = logical(bitxor(double(t_best),1));

            err(t_best) = error_t(t_best);
            err(p_best) = error_p(p_best);
        end

        function err = local_error(obj,q,qd)
            % Calculates the error in the robot's local frame
            % Rotation matrix
            R = @(theta) [cos(theta),sin(theta),0;-sin(theta),cos(theta),0;0,0,1];
            theta = q(3);
            ep = (qd - q)';
            ep(3) = obj.angle_error(qd(3),q(3));
            err = R(theta)*ep;
        end

        function u = feedback_vel(obj,omegad,vd,err)
            % Uses err to get a better estimate of the linear and angular velocities 
            Zeta = 0.99;
            g = 32*0.0005;

            omega_n = sqrt(omegad^2 + g*vd^2);
            k1 = 2*Zeta*omega_n;
            k2 = g*abs(vd);
            k3 = k1;
            
            Ks = [k1,0,0;0,sign(vd)*k2,k3]; % Gain matrix
            
            uB = Ks*err;
            uF = [vd*cos(err(3));omegad];
            if obj.useFeedback
                u = uB + uF;
                %u(1) = abs(u(1));
            else
                u = [vd;omegad];
            end
        end

    end

    methods(Access = protected)
        function setupImpl(obj)
            % This function is called when the simulink simulation starts
            % Estimates the angular and linear velocities of the reference path
            obj.ii = 1;
            obj.T = obj.traj.tSamples(2);
            %obj.SampleTime = obj.T;
            %[obj.OMEGAD, obj.VD, obj.THETAD] = obj.estimate_state_pos( ...
            %    obj.traj.q(1,:),obj.traj.q(2,:),obj.traj.qd(1,:),obj.traj.qd(2,:),obj.T);
            [obj.OMEGAD, obj.VD, obj.THETAD] = obj.estimate_state_accel( ...
                obj.traj.qd(1,:),obj.traj.qd(2,:),obj.traj.qdd(1,:),obj.traj.qdd(2,:));
            obj.N = length(obj.THETAD);
            obj.THETAD = obj.normalize_angle(obj.THETAD);
            % Give the first estimate
            obj.q_est = [obj.traj.q(1,obj.ii),obj.traj.q(2,obj.ii),obj.THETAD(obj.ii)];
            % Set the sample time to obj.SampleTime
            sts = getSampleTime(obj);
        end

        function [v,omega,xd,yd] = stepImpl(obj,x,y,theta)
            % This function is executed at each time step defined previously
            % As long as there's a path it tries to follow it as close as possible
            % When the path ends it just set the angular and linear velocities to zero
            % The position and orientation of the robot can be estimated internally
            % using the obj.kin_model function, 
            % or it can be given externally from the simulink simulation.
            % 
            
            % Use the positions and orientations given from the simulation
            if not(obj.useKinematicModel)
                theta = obj.normalize_angle(theta);
                obj.q_est = [x,y,theta];
            end

            if (obj.ii < obj.N)
                omegad = obj.OMEGAD(obj.ii);
                vd = obj.VD(obj.ii);
        
                q_ref = [obj.traj.q(:,obj.ii).',obj.THETAD(obj.ii)];
                err = obj.local_error(obj.q_est,q_ref);
                u = obj.feedback_vel(omegad,vd,err);
                v = u(1);
                omega = u(2);
                obj.ii = obj.ii + 1;

                % Estimate next state if using the kinematic model
                if obj.useKinematicModel
                    obj.q_est = obj.q_est + obj.kin_model(u(2),u(1),obj.q_est(3),obj.T);
                    obj.q_est(3) = obj.normalize_angle(obj.q_est(3));
                else
                    obj.q_est = [0,0,0];
                end
                xd = q_ref(1);
                yd = q_ref(2);
            else
                % Set the velocities to zero
                q_ref = [obj.traj.q(:,obj.ii).',obj.THETAD(obj.ii)];
                v = 0;
                omega = 0;
                xd = q_ref(1);
                yd = q_ref(2);
            end
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            obj.ii = 1;
            obj.q_est = [obj.traj.q(1,obj.ii),obj.traj.q(2,obj.ii),obj.THETAD(obj.ii)];
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
