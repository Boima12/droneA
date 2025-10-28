classdef trajectory_planner
    % Class generates reference trajectories (timeseries for Simulink)
    
    properties
        Tsim = 15;    % simulation time
        dt   = 0.01;  % time step
        waypoints      % struct of waypoints {x,y,z,t}
    end
    
    methods
        function obj = trajectory_planner(Tsim, dt)
            if nargin > 0
                obj.Tsim = Tsim;
                obj.dt = dt;
            end
        end
        
        function obj = defaultWaypoints(obj)
            % Define default (ascending then forward)
            obj.waypoints = struct( ...
                'x', [0 10], ...
                'y', [0 5], ...
                'z', [0 5], ...
                't', [0 obj.Tsim]);
        end
        
        function [x_ref, y_ref, z_ref, t] = generate(obj)
            % Generate smooth trajectory from waypoints using spline
            if isempty(obj.waypoints)
                obj = obj.defaultWaypoints();
            end
            t = (0:obj.dt:obj.Tsim)';
            x_ref = timeseries(spline(obj.waypoints.t, obj.waypoints.x, t), t);
            y_ref = timeseries(spline(obj.waypoints.t, obj.waypoints.y, t), t);
            z_ref = timeseries(spline(obj.waypoints.t, obj.waypoints.z, t), t);
        end
    end
end
