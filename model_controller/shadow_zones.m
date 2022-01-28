classdef shadow_zones < matlab.System
    % Untitled Add summary here
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.

    % Public, tunable properties
    properties
        oc_matx = struct
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

        function z = stepImpl(obj,x,y)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            
            k = 0.1852;
            x_mat = x*k;
            y_mat = y*k;
            
            size(obj.oc_matx.oc_mat)
            
            if (obj.oc_matx.oc_mat(round(y_mat),round(x_mat)) == 3)
                z = 0;
            else
                z = 1;
            end


        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
    end
end
