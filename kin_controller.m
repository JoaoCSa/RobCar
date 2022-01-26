classdef kin_controller < matlab.System
    % untitled Add summary here
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.

    % Public, tunable properties
    properties
        traj = struct
        OffsetTime = 0.0 % Offset Time
        SampleTime = 0.1 % Sample Time
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
        function obj = kin_controller(trajectory)
            if nargin > 0
                obj.traj = trajectory
            end
        end

        function [OMEGA,V,THETA] = estimate_state_accel(obj,X_dot,Y_dot,X_dotdot,Y_dotdot,T)
            V_squared = X_dot.^2 + Y_dot.^2;

            THETA = atan2(Y_dot,X_dot);
            V = sqrt(V_squared);
            OMEGA = (X_dot.*Y_dotdot - Y_dot.*X_dotdot)./V_squared;
        end

        function q_dot = kin_model(obj,omega,v,theta,T)
            % Discrete time car kinematics
            % It assumes a constant linear velocity and angular velocity
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
            p_best = logical(bitxor(t_best,1));

            err(t_best) = error_t(t_best);
            err(p_best) = error_p(p_best);
        end

        function err = local_error(obj,q,qd)
            R = @(theta) [cos(theta),sin(theta),0;-sin(theta),cos(theta),0;0,0,1];
            theta = q(3);
            ep = (qd - q)';
            err = R(theta)*ep;
        end

        function u = feedback_vel(obj,omegad,vd,err)
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

    end

    methods(Access = protected)
        function setupImpl(obj)
            obj.ii = 1;
            obj.T = obj.traj.tSamples(2);
            %obj.SampleTime = obj.T;
            obj.N = length(obj.traj.tSamples);
            [obj.OMEGAD, obj.VD, obj.THETAD] = obj.estimate_state_accel(obj.traj.qd(1,:),obj.traj.qd(2,:),obj.traj.qdd(1,:),obj.traj.qdd(2,:),obj.T);
            obj.THETAD = obj.normalize_angle(obj.THETAD);
            obj.q_est = [obj.traj.q(1,obj.ii),obj.traj.q(2,obj.ii),obj.THETAD(obj.ii)];
            sts = getSampleTime(obj);
            % Perform one-time calculations, such as computing constants
        end

        function [v,omega] = stepImpl(obj,time)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.

            if (obj.ii < obj.N)
                omegad = obj.OMEGAD(obj.ii);
                vd = obj.VD(obj.ii);
        
                q_ref = [obj.traj.q(:,obj.ii).',obj.THETAD(obj.ii)];
                err = obj.local_error(obj.q_est,q_ref);
                u = obj.feedback_vel(omegad,vd,err);
                v = u(1);
                omega = u(2);
                obj.ii = obj.ii + 1;
                % Estimate next state
                obj.q_est = obj.q_est + obj.kin_model(u(2),u(1),obj.q_est(3),obj.T);
            else
                v = 0;
                omega = 0;
            end
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            obj.ii = 1;
            obj.q_est = [obj.traj.q(1,obj.ii),obj.traj.q(2,obj.ii),obj.THETAD(obj.ii)];
        end

        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj,'Type','Discrete',...
                'SampleTime',obj.SampleTime, ...
                'OffsetTime',obj.OffsetTime);
        end

        function flag = isInactivePropertyImpl(obj,prop)
            flag = false;
    
            if any(strcmp(prop,{'TickTime'}))
                flag = true;
            end
        end

    end
end
