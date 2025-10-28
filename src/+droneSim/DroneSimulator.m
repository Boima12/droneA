classdef DroneSimulator
    % Wrapper class for running Simulink drone models
    
    properties
        model = 'models/quadcopter_simple';
        Tsim = 15;
    end
    
    methods
        function obj = DroneSimulator(model, Tsim)
            if nargin > 0
                obj.model = model;
                obj.Tsim = Tsim;
            end
        end
        
        function simOut = run(obj, x_ref, y_ref, z_ref)
            assignin('base', 'x_ref', x_ref);
            assignin('base', 'y_ref', y_ref);
            assignin('base', 'z_ref', z_ref);
            open_system(obj.model);
            simOut = sim(obj.model, 'StopTime', num2str(obj.Tsim));
        end
    end
    
    methods (Static)
        function plotResults(simOut, x_ref, y_ref, z_ref)
            % Try to extract outputs
            try
                x_ts = simOut.get('x_out');
                y_ts = simOut.get('y_out');
                z_ts = simOut.get('z_out');
            catch
                if evalin('base','exist(''x_out'',''var'')')
                    x_ts = evalin('base','x_out');
                    y_ts = evalin('base','y_out');
                    z_ts = evalin('base','z_out');
                else
                    error('Cannot find simulation outputs x_out/y_out/z_out.');
                end
            end
        
            % Convert to numeric arrays
            if isa(x_ts,'timeseries')
                t = x_ts.Time; 
                x = x_ts.Data; 
                y = y_ts.Data; 
                z = z_ts.Data;
            else
                t = x_ts.time; 
                x = x_ts.signals.values;
                y = y_ts.signals.values;
                z = z_ts.signals.values;
            end
        
            % === Ensure same time resolution ===
            % Interpolate reference signals to match simulation time
            x_ref_interp = interp1(x_ref.Time, x_ref.Data, t, 'linear', 'extrap');
            y_ref_interp = interp1(y_ref.Time, y_ref.Data, t, 'linear', 'extrap');
            z_ref_interp = interp1(z_ref.Time, z_ref.Data, t, 'linear', 'extrap');
        
            % === Plot 3D Trajectory ===
            figure('Name','3D Trajectory');
            plot3(x, y, z, 'b', 'LineWidth', 1.5); hold on;
            plot3(x_ref_interp, y_ref_interp, z_ref_interp, 'r--', 'LineWidth', 1);
            grid on; axis equal;
            xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
            legend('Actual', 'Reference');
            title('Drone 3D Trajectory');
        
            % === Plot each axis vs time ===
            figure('Name','Position vs Time');
            subplot(3,1,1); plot(t, x, 'b', t, x_ref_interp, 'r--'); ylabel('x (m)'); grid on; legend('Actual','Ref');
            subplot(3,1,2); plot(t, y, 'b', t, y_ref_interp, 'r--'); ylabel('y (m)'); grid on; legend('Actual','Ref');
            subplot(3,1,3); plot(t, z, 'b', t, z_ref_interp, 'r--'); ylabel('z (m)'); xlabel('Time (s)'); grid on; legend('Actual','Ref');
        
            disp('✅ Plot complete.');
        end
    end
end
