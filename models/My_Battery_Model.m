clear; clc; close all; bdclose('all');

model_name = 'Battery';

if bdIsLoaded(model_name)
    close_system(model_name, 0);
end

new_system(model_name);
open_system(model_name);

% Đoạn 2: Setpoint
add_block('simulink/Sources/Step', [model_name '/Setpoint']);
set_param([model_name '/Setpoint'], ...
    'Position', [100 150 130 170], ...
    'Time', '0', ...
    'Before', '0', ...
    'After', '1');

% Đoạn 3: Sum, PID, Plant, Scope
add_block('simulink/Math Operations/Sum', [model_name '/Sum']);
set_param([model_name '/Sum'], 'Position', [250 140 270 180], 'Inputs', '|+-');

add_block('simulink/Continuous/PID Controller', [model_name '/PID']);
set_param([model_name '/PID'], 'Position', [350 140 410 180]);

add_block('simulink/Continuous/Transfer Fcn', [model_name '/Plant']);
set_param([model_name '/Plant'], 'Position', [480 140 540 180], ...
          'Numerator', '[1]', 'Denominator', '[10 1]');

add_block('simulink/Sinks/Scope', [model_name '/Scope']);
set_param([model_name '/Scope'], 'Position', [600 130 630 190], 'NumInputPorts', '2');

% Đoạn 4: Kết nối
add_line(model_name, 'Setpoint/1', 'Sum/1');
add_line(model_name, 'Plant/1', 'Sum/2');
add_line(model_name, 'Sum/1', 'PID/1');
add_line(model_name, 'PID/1', 'Plant/1');
add_line(model_name, 'Setpoint/1', 'Scope/1');
add_line(model_name, 'Plant/1', 'Scope/2');

% Đoạn 5: Tune PID + Lưu
sys = tf(1, [10 1]);
C = pidtune(sys, 'pid');
set_param([model_name '/PID'], 'P', num2str(C.Kp), 'I', num2str(C.Ki), 'D', num2str(C.Kd));

set_param(model_name, 'StopTime', '50');
save_system(model_name);
disp('ĐÃ TẠO FILE: Battery.slx');

% Đoạn 6: Chạy + Vẽ
simOut = sim(model_name);
open_system([model_name '/Scope']);

figure;
plot(simOut.scopeData.time, simOut.scopeData.signals(1).values, 'b', 'LineWidth', 2); hold on;
plot(simOut.scopeData.time, simOut.scopeData.signals(2).values, 'r--', 'LineWidth', 2);
grid on; legend('Setpoint', 'Output'); xlabel('Thời gian'); ylabel('Mức');
title('Mô hình Battery - PID điều khiển bình');