classdef trajectory_planner
    % Generates different reference trajectories (for Simulink)
    
    properties
        Tsim = 15;    % simulation time [s]
        dt   = 0.01;  % time step [s]
        waypoints      % optional struct of waypoints {x,y,z,t}
    end
    
    methods
        function obj = trajectory_planner(Tsim, dt)
            if nargin > 0
                obj.Tsim = Tsim;
                obj.dt = dt;
            end
        end
        
        %% Default waypoint-based trajectory
        function obj = defaultWaypoints(obj)
            obj.waypoints = struct( ...
                'x', [0 10], ...
                'y', [0 5], ...
                'z', [0 5], ...
                't', [0 obj.Tsim]);
        end
        
        function [x_ref, y_ref, z_ref, t] = generate(obj)
            if isempty(obj.waypoints)
                obj = obj.defaultWaypoints();
            end
            t = (0:obj.dt:obj.Tsim)';
            x_ref = timeseries(spline(obj.waypoints.t, obj.waypoints.x, t), t);
            y_ref = timeseries(spline(obj.waypoints.t, obj.waypoints.y, t), t);
            z_ref = timeseries(spline(obj.waypoints.t, obj.waypoints.z, t), t);
        end

        %% 1️⃣ Circular trajectory
        function [x_ref, y_ref, z_ref, t] = generateCircle(obj, radius, height)
            if nargin < 2, radius = 5; end
            if nargin < 3, height = 3; end
            t = (0:obj.dt:obj.Tsim)';
            omega = 2*pi/obj.Tsim; % one revolution
            x = radius * cos(omega * t);
            y = radius * sin(omega * t);
            z = ones(size(t)) * height;
            x_ref = timeseries(x, t);
            y_ref = timeseries(y, t);
            z_ref = timeseries(z, t);
            obj.plotTrajectory(x, y, z, 'Circular trajectory');
        end

        %% 2️⃣ Figure-8 trajectory
        function [x_ref, y_ref, z_ref, t] = generateFigure8(obj, radius, height)
            if nargin < 2, radius = 5; end
            if nargin < 3, height = 3; end
            t = (0:obj.dt:obj.Tsim)';
            omega = 2*pi/obj.Tsim;
            x = radius * sin(omega * t);
            y = radius * sin(omega * t) .* cos(omega * t);
            z = ones(size(t)) * height;
            x_ref = timeseries(x, t);
            y_ref = timeseries(y, t);
            z_ref = timeseries(z, t);
            obj.plotTrajectory(x, y, z, 'Figure-8 trajectory');
        end

        %% 3️⃣ 3D Wave trajectory
        function [x_ref, y_ref, z_ref, t] = generateWave3D(obj, amplitude, wavelength)
            if nargin < 2, amplitude = 2; end
            if nargin < 3, wavelength = 3; end
            t = (0:obj.dt:obj.Tsim)';
            x = linspace(0, 10, numel(t))';
            y = amplitude * sin(2*pi*x/wavelength);
            z = 3 + amplitude * cos(2*pi*x/wavelength);
            x_ref = timeseries(x, t);
            y_ref = timeseries(y, t);
            z_ref = timeseries(z, t);
            obj.plotTrajectory(x, y, z, '3D Wave trajectory');
        end
        
        %% Plot helper
        function plotTrajectory(~, x, y, z, titleStr)
            figure('Name', titleStr);
            plot3(x, y, z, 'LineWidth', 1.8);
            grid on; axis equal;
            xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
            title(titleStr);
        end
    end
end
