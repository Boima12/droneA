% Mô phỏng Drone 3D với PID và Quỹ đạo trong MATLAB
clear all; close all; clc;

% Tham số mô phỏng
dt = 0.01; % Bước thời gian (s)
T = 10;    % Thời gian mô phỏng (s)
time = 0:dt:T;
n_steps = length(time);

% Lập kế hoạch quỹ đạo
freq = 0.5; % Tần số cho quỹ đạo sin (Hz)
traj_x = sin(2 * pi * freq * time); % Quỹ đạo sin cho x
traj_y = ones(1, n_steps); % Vị trí y cố định (trước-sau)
traj_z = linspace(0, 2, n_steps); % Tăng tuyến tính từ 0 đến 2m cho z (độ cao)

% Trạng thái ban đầu của drone
pos_x = 0; vel_x = 0;
pos_y = 0; vel_y = 0;
pos_z = 0; vel_z = 0;

% Tham số PID (tinh chỉnh sơ bộ)
kp_x = 2.0; ki_x = 0.5; kd_x = 1.0;
kp_y = 2.0; ki_y = 0.5; kd_y = 1.0;
kp_z = 2.0; ki_z = 0.5; kd_z = 1.0;

% Khởi tạo biến lưu trữ cho PID
integral_x = 0; prev_error_x = 0;
integral_y = 0; prev_error_y = 0;
integral_z = 0; prev_error_z = 0;

% Mảng lưu trữ kết quả mô phỏng
pos_sim_x = zeros(1, n_steps);
pos_sim_y = zeros(1, n_steps);
pos_sim_z = zeros(1, n_steps);

% Vòng lặp mô phỏng
for i = 1:n_steps
    % Tính sai số
    error_x = traj_x(i) - pos_x;
    error_y = traj_y(i) - pos_y;
    error_z = traj_z(i) - pos_z;
    
    % PID cho trục x
    integral_x = integral_x + error_x * dt;
    derivative_x = (error_x - prev_error_x) / dt;
    a_x = kp_x * error_x + ki_x * integral_x + kd_x * derivative_x;
    a_x = max(min(a_x, 10), -10); % Giới hạn gia tốc
    
    % PID cho trục y
    integral_y = integral_y + error_y * dt;
    derivative_y = (error_y - prev_error_y) / dt;
    a_y = kp_y * error_y + ki_y * integral_y + kd_y * derivative_y;
    a_y = max(min(a_y, 10), -10); % Giới hạn gia tốc
    
    % PID cho trục z
    integral_z = integral_z + error_z * dt;
    derivative_z = (error_z - prev_error_z) / dt;
    a_z = kp_z * error_z + ki_z * integral_z + kd_z * derivative_z;
    a_z = max(min(a_z, 10), -10); % Giới hạn gia tốc
    
    % Cập nhật trạng thái
    vel_x = vel_x + a_x * dt;
    pos_x = pos_x + vel_x * dt;
    vel_y = vel_y + a_y * dt;
    pos_y = pos_y + vel_y * dt;
    vel_z = vel_z + a_z * dt;
    pos_z = pos_z + vel_z * dt;
    
    % Lưu trạng thái
    pos_sim_x(i) = pos_x;
    pos_sim_y(i) = pos_y;
    pos_sim_z(i) = pos_z;
    
    % Cập nhật sai số trước đó
    prev_error_x = error_x;
    prev_error_y = error_y;
    prev_error_z = error_z;
end

% In kết quả
fprintf('Vị trí cuối x: %.3f (mục tiêu: %.3f)\n', pos_x, traj_x(end));
fprintf('Vị trí cuối y: %.3f (mục tiêu: %.3f)\n', pos_y, traj_y(end));
fprintf('Vị trí cuối z: %.3f (mục tiêu: %.3f)\n', pos_z, traj_z(end));
fprintf('Sai số theo dõi tối đa x: %.3f m\n', max(abs(pos_sim_x - traj_x)));
fprintf('Sai số theo dõi tối đa y: %.3f m\n', max(abs(pos_sim_y - traj_y)));
fprintf('Sai số theo dõi tối đa z: %.3f m\n', max(abs(pos_sim_z - traj_z)));

% Vẽ đồ thị theo thời gian
figure('Name', 'Kết quả mô phỏng quỹ đạo Drone 3D');
subplot(3, 1, 1);
plot(time, traj_x, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Mục tiêu x');
hold on;
plot(time, pos_sim_x, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Thực tế x');
title('Quỹ đạo trục X'); xlabel('Thời gian (s)'); ylabel('Vị trí X (m)');
legend; grid on;

subplot(3, 1, 2);
plot(time, traj_y, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Mục tiêu y');
hold on;
plot(time, pos_sim_y, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Thực tế y');
title('Quỹ đạo trục Y'); xlabel('Thời gian (s)'); ylabel('Vị trí Y (m)');
legend; grid on;

subplot(3, 1, 3);
plot(time, traj_z, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Mục tiêu z');
hold on;
plot(time, pos_sim_z, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Thực tế z');
title('Quỹ đạo trục Z (Độ cao)'); xlabel('Thời gian (s)'); ylabel('Vị trí Z (m)');
legend; grid on;

% Vẽ quỹ đạo 3D
figure('Name', 'Quỹ đạo 3D của Drone');
plot3(traj_x, traj_y, traj_z, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Quỹ đạo mục tiêu');
hold on;
plot3(pos_sim_x, pos_sim_y, pos_sim_z, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Quỹ đạo thực tế');
title('Quỹ đạo Drone trong không gian 3D');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
legend; grid on; axis equal;