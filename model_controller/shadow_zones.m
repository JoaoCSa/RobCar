classdef shadow_zones < matlab.System
    % No GPS zones
    
    % Public, tunable properties
    properties
        oc_matx = struct
        d = 0.64;
        L = 2.2;
        Lr = 0.566;
        Lf = 0.566;
        k = 0.1852;
    end

    properties(DiscreteState)

    end

    % Pre-computed constants
    properties(Access = private)

    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
        end

        function [z, c_check, x_col, y_col] = stepImpl(obj,x,y,theta)
            % Search x and y in green matrix
            % theta em rad
            
            x_mat = x/obj.k;
            y_mat = y/obj.k;
            c_mtx = [0;0;0;0];
            
            % 3 + (5)*cos(deg2rad(180)) + 2*sin(deg2rad(180))
            % 3 + (5)*sin(deg2rad(180)) + 2*cos(deg2rad(180))
            
            % 3 - ((3)*cos(deg2rad(-90)) + 2*sin(deg2rad(-90)))
            % 3 - ((3)*sin(deg2rad(180)) + 2*cos(deg2rad(180)))
            
            x_front_L = (x + (obj.L + obj.Lf)*cos(theta) - obj.d*sin(theta))/obj.k;
            y_front_L = (y + (obj.L + obj.Lf)*sin(theta) + obj.d*cos(theta))/obj.k;
            x_back_L = (x - ((obj.Lr)*cos(theta) + obj.d*sin(theta)))/obj.k;
            y_back_L = (y - ((obj.Lr)*sin(theta) - obj.d*cos(theta)))/obj.k;
            
            
            x_front_R = (x + (obj.L + obj.Lf)*cos(theta) + obj.d*sin(theta))/obj.k;
            y_front_R = (y + (obj.L + obj.Lf)*sin(theta) - obj.d*cos(theta))/obj.k;
            x_back_R = (x - ((obj.Lr)*cos(theta) - obj.d*sin(theta)))/obj.k;
            y_back_R = (y - ((obj.Lr)*sin(theta) + obj.d*cos(theta)))/obj.k;
            
            %% Shadow zones (no GPS)
            
            if (obj.oc_matx.occ_matrix(round(y_mat),round(x_mat)) == 3)
                z = 0;
            else
                z = 1;
            end
            
            
            %% Collisions
            
            % c_mtx = [front_L; front_R; back_L; back_R]
            
            c_mtx = [obj.oc_matx.occ_matrix(round(y_front_L),round(x_front_L));
                     obj.oc_matx.occ_matrix(round(y_front_R),round(x_front_R));
                     obj.oc_matx.occ_matrix(round(y_back_L),round(x_back_L));
                     obj.oc_matx.occ_matrix(round(y_back_R),round(x_back_R))]
            
            if (c_mtx(1) ~= 1 && c_mtx(2) ~= 1 && c_mtx(3) ~= 1 && c_mtx(4) ~= 1) % no collision 
                c_check = 0;
                x_col = NaN;
                y_col = NaN;
            elseif (c_mtx(1) == 1 && c_mtx(2) ~= 1 && c_mtx(3) ~= 1 && c_mtx(4) ~= 1) % collision front Left
                c_check = 1;
                x_col = x;
                y_col = y;
            elseif (c_mtx(1) ~= 1 && c_mtx(2) == 1 && c_mtx(3) ~= 1 && c_mtx(4) ~= 1) % collision front Right
                c_check = 2;
                x_col = x;
                y_col = y;
            elseif (c_mtx(1) ~= 1 && c_mtx(2) ~= 1 && c_mtx(3) == 1 && c_mtx(4) ~= 1) % collision back Left
                c_check = 3;
                x_col = x;
                y_col = y;
            elseif (c_mtx(1) ~= 1 && c_mtx(2) ~= 1 && c_mtx(3) ~= 1 && c_mtx(4) == 1) % collision back Right
                c_check = 4;
                x_col = x;
                y_col = y;
            else % multiple collisions
                c_check = 5;
                x_col = x;
                y_col = y;
            end
            
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
    end
end
