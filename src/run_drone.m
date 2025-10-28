%% run_drone.m
clear; close all; clc;

% Load parameters
run('src/params.m');

% Create trajectory planner
plannerObj = planner.trajectory_planner(15, 0.01);
[x_ref, y_ref, z_ref, t] = plannerObj.generate();

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