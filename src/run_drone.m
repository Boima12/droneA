%% run_drone.m
% Initialize parameters and run the simple 3-DOF drone Simulink model
clear; close all; clc;

% Reference trajectory over time
Tsim = 15;   % simulation time in seconds
t = linspace(0, Tsim, 300)';  % 300 samples

% Define trajectory:
x_ref_values = zeros(size(t));
y_ref_values = zeros(size(t));
z_ref_values = zeros(size(t));

% Phase 1: ascend from z=0 to z=5 in first 5 seconds
idx1 = t <= 5;
z_ref_values(idx1) = 5 * (t(idx1) / 5);

% Phase 2: move forward (x: 0→10, y: 0→5) from 5–15s
idx2 = t > 5;
x_ref_values(idx2) = 10 * (t(idx2) - 5) / (Tsim - 5);
y_ref_values(idx2) = 5 * (t(idx2) - 5) / (Tsim - 5);
z_ref_values(idx2) = 5;  % maintain altitude

% Create timeseries objects for Simulink
x_ref = timeseries(x_ref_values, t);
y_ref = timeseries(y_ref_values, t);
z_ref = timeseries(z_ref_values, t);

assignin('base','x_ref', x_ref);
assignin('base','y_ref', y_ref);
assignin('base','z_ref', z_ref);


% Name of your model
model = 'quadcopter_simple';
open_system(model);

% Run simulation
simOut = sim(model,'StopTime',num2str(Tsim));

% Depending on To Workspace setting, retrieve signals:
% If To Workspace saved as Structure with Time:
try
    x_ts = simOut.get('x_out');  % timeseries
    y_ts = simOut.get('y_out');
    z_ts = simOut.get('z_out');
catch
    % Alternative: variables in base workspace
    if evalin('base','exist(''x_out'',''var'')')
        x_ts = evalin('base','x_out');
        y_ts = evalin('base','y_out');
        z_ts = evalin('base','z_out');
    else
        error('Cannot find simulation outputs x_out/y_out/z_out. Check To Workspace settings.');
    end
end

% Convert to arrays for plotting if necessary
if isa(x_ts,'timeseries'); tx = x_ts.Time; xdata = x_ts.Data; else tx = x_ts.time; xdata = x_ts.signals.values; end
if isa(y_ts,'timeseries'); ty = y_ts.Time; ydata = y_ts.Data; else ty = y_ts.time; ydata = y_ts.signals.values; end
if isa(z_ts,'timeseries'); tz = z_ts.Time; zdata = z_ts.Data; else tz = z_ts.time; zdata = z_ts.signals.values; end

% If times are identical, use one
t = tx;

% Plot 3D trajectory
figure('Name','3D Trajectory');
plot3(xdata, ydata, zdata, '-o','LineWidth',1.5, 'MarkerSize',3);
grid on; xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Drone 3D Trajectory'); axis equal;

% Plot each axis vs time
figure('Name','Position vs Time');
subplot(3,1,1); plot(t, xdata); ylabel('x (m)'); grid on; title('X vs time');
subplot(3,1,2); plot(t, ydata); ylabel('y (m)'); grid on; title('Y vs time');
subplot(3,1,3); plot(t, zdata); ylabel('z (m)'); xlabel('Time (s)'); grid on; title('Z vs time');

% Save figures (optional)
savefig('trajectory.fig');
saveas(gcf,'position_vs_time.png');

disp('Simulation complete. Figures saved.');
