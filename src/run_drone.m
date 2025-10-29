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

% Tạo đối tượng pin
battery = components.BatteryModel('Capacity_Ah', 2.2, ...
                                  'NominalVoltage', 11.1, ...
                                  'dt', 0.01);

% === Lấy dữ liệu dòng điện từ Simulink ===
try
    I_ts = simOut.get('I_total');
    I_data = I_ts.Data';
    t_I = I_ts.Time';
    disp('✅ Found I_total from Simulink output.');
catch
    warning('⚠️ Could not find I_total in Simulink output. Using 5A default.');
    I_data = 5;
    t_I = 0;
end

% === Tạo trục thời gian cho battery ===
t_batt = 0:battery.dt:simObj.Tsim;

% === Nếu I_data là vector, nội suy theo thời gian pin ===
if numel(I_data) > 1
    I_interp = interp1(t_I, I_data, t_batt, 'linear', 'extrap');
else
    I_interp = I_data * ones(size(t_batt));
end

% === Mô phỏng ===
[t_batt, voltage, soc] = battery.simulate(simObj.Tsim, I_interp);

% === Vẽ kết quả ===
battery.plotResults(t_batt, voltage, soc);
disp('✅ Battery simulation complete.');

%% Yaw + Attitude PID Simulation
disp('--- Running Yaw + Attitude PID simulation ---');
controller = control.YawAttitudePID();
controller.simulate();
disp('✅ Yaw + Attitude PID simulation complete.');