%% run_drone.m
clear; close all; clc;

% Load parameters
run('src/params.m');

% Create trajectory planner
plannerObj = planner.trajectory_planner(15, 0.01);

% Circle
[x_ref, y_ref, z_ref, t] = plannerObj.generateCircle(5, 3);
% Figure-8
%[x_ref, y_ref, z_ref, t] = plannerObj.generateFigure8(5, 3);
% Wave 3D
%[x_ref, y_ref, z_ref, t] = plannerObj.generateWave3D(2, 3);



% Run simulation
simObj = droneSim.DroneSimulator('models/quadcopter_simple', 15);
simOut = simObj.run(x_ref, y_ref, z_ref);

% Plot results
droneSim.DroneSimulator.plotResults(simOut, x_ref, y_ref, z_ref);


%% Battery simulation
disp('--- Running battery simulation ---');
battery = components.BatteryModel('Capacity_Ah', 2.2, 'NominalVoltage', 11.1);

% Giả định dòng tiêu thụ 5A trong suốt 15s
[t_batt, voltage, soc] = battery.simulate(15, 5);

battery.plotResults(t_batt, voltage, soc);
disp('✅ Battery simulation complete.');


%% Yaw + Attitude PID Simulation
disp('--- Running Yaw + Attitude PID simulation ---');
controller = control.YawAttitudePID();
controller.simulate();
disp('✅ Yaw + Attitude PID simulation complete.');


%% 3D Visualization
disp('--- Running 3D visualization animation ---');
anim = visual.Drone3DAnimator();
anim.animate(t, x_ref.Data, y_ref.Data, z_ref.Data);
disp('✅ 3D animation complete.');
