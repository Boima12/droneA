% MÔ PHỎNG BỘ ĐIỀU KHIỂN YAW + ATTITUDE PID (vòng lặp lồng nhau)
% cho mô hình quadrotor (drone 4 cánh) đơn giản.
%
% Bao gồm:
%  - Outer loop: PID điều khiển góc (roll, pitch, yaw)
%  - Inner loop: PID điều khiển tốc độ quay (p, q, r)
%  - Mô hình động học quay: I*omega_dot + omega x (I*omega) = tau
%  - Hiển thị đồ thị + mô phỏng 3D drone

close all;
clear;
clc;

%% ==== THÔNG SỐ MÔ PHỎNG ====
dt = 0.001;           % bước thời gian (s)
Tsim = 8;             % tổng thời gian mô phỏng (s)
t = 0:dt:Tsim;        % vector thời gian

%% ==== THÔNG SỐ ĐỘNG HỌC ====
% Ma trận quán tính I (kg*m^2) – mô phỏng drone có kích thước không đối xứng
I = diag([0.02, 0.025, 0.03]); % [Ixx, Iyy, Izz]

%% ==== TRẠNG THÁI BAN ĐẦU ====
omega = [0;0;0];      % tốc độ góc [p; q; r] (rad/s)
angles = [0;0;0];     % góc Euler [roll; pitch; yaw] (rad)

%% ==== GIÁ TRỊ ĐẶT (SETPOINTS) ====
% Góc mong muốn (độ), thay đổi sau 1 giây
roll_sp_deg  = 10;    % Roll = nghiêng quanh trục X
pitch_sp_deg = -5;    % Pitch = nghiêng quanh trục Y
yaw_sp_deg   = 30;    % Yaw = quay quanh trục Z

% Ban đầu (trước 1s) drone đứng yên, không góc nghiêng
roll_sp_init  = 0;
pitch_sp_init = 0;
yaw_sp_init   = 0;

%% ==== THAM SỐ PID (VÒNG NGOÀI - GÓC) ====
% Điều khiển attitude: cho biết tốc độ góc mong muốn (rate_sp)
Kp_ang = [6, 6, 4];   % hệ số tỉ lệ (roll, pitch, yaw)
Ki_ang = [0.5, 0.5, 0.2];
Kd_ang = [1.0, 1.0, 0.6];

%% ==== THAM SỐ PID (VÒNG TRONG - TỐC ĐỘ) ====
% Điều khiển rate: cho ra mô-men lực (tau)
Kp_rate = [0.12, 0.12, 0.08];
Ki_rate = [0.01, 0.01, 0.005];
Kd_rate = [0.002, 0.002, 0.001];

%% ==== GIỚI HẠN VÀ RÀNG BUỘC ====
rate_sp_max = deg2rad([200, 200, 120]); % giới hạn tốc độ góc mong muốn (rad/s)
torque_limit = [0.15, 0.15, 0.15];      % giới hạn mô-men điều khiển (N*m)

%% ==== KHỞI TẠO BIẾN LƯU ====
N = length(t);
angles_hist = zeros(3,N);   % lưu lịch sử góc
omega_hist  = zeros(3,N);   % lưu lịch sử tốc độ góc
tau_hist    = zeros(3,N);   % lưu moment
angle_sp_hist = zeros(3,N); % lưu setpoint góc

%% ==== KHỞI TẠO BỘ NHỚ PID ====
int_ang = zeros(3,1);       % tích phân vòng ngoài
prev_err_ang = zeros(3,1);
int_rate = zeros(3,1);      % tích phân vòng trong
prev_err_rate = zeros(3,1);

%% ==== NHIỄU NGOÀI (GÓI GIÓ) ====
% Nhiễu torque tác động trong 0.5s (3s–3.5s)
disturbance = @(ti) (ti>3 && ti<3.5).* [0.02; -0.015; 0.03];

%% ==== HÌNH DẠNG DRONE ====
% Dùng để vẽ mô phỏng 3D (thân và tay drone)
bodyW = 0.18; bodyL = 0.18; armLen = 0.14;
[bodyV, bodyF] = makeBox(bodyL, bodyW, 0.03);

%% ==== VÒNG LẶP MÔ PHỎNG CHÍNH ====
for k=1:N
    ti = t(k);

    % --- Xác định góc mong muốn (setpoint) ---
    if ti < 1
        sp = deg2rad([roll_sp_init; pitch_sp_init; yaw_sp_init]);
    else
        sp = deg2rad([roll_sp_deg; pitch_sp_deg; yaw_sp_deg]);
    end

    % --- VÒNG NGOÀI: Angle PID ---
    % → Tính sai số góc và cho ra tốc độ góc mong muốn (rate_sp)
    err_ang = sp - angles;
    int_ang = int_ang + err_ang * dt;
    derr_ang = (err_ang - prev_err_ang) / dt;
    rate_sp = Kp_ang'.*err_ang + Ki_ang'.*int_ang + Kd_ang'.*derr_ang;

    % Giới hạn tốc độ mong muốn
    rate_sp = max(min(rate_sp, rate_sp_max'), -rate_sp_max');
    prev_err_ang = err_ang;

    % --- VÒNG TRONG: Rate PID ---
    % → Tính sai số tốc độ quay và tạo moment điều khiển (tau)
    err_rate = rate_sp - omega;
    int_rate = int_rate + err_rate * dt;
    derr_rate = (err_rate - prev_err_rate) / dt;
    tau = Kp_rate'.*err_rate + Ki_rate'.*int_rate + Kd_rate'.*derr_rate;
    prev_err_rate = err_rate;

    % Giới hạn moment và chống tích phân quá mức (anti-windup)
    for i=1:3
        if tau(i) > torque_limit(i)
            tau(i) = torque_limit(i);
            int_rate(i) = int_rate(i) - err_rate(i)*dt;
        elseif tau(i) < -torque_limit(i)
            tau(i) = -torque_limit(i);
            int_rate(i) = int_rate(i) - err_rate(i)*dt;
        end
    end

    % --- Thêm nhiễu bên ngoài ---
    tau = tau + disturbance(ti);

    % --- MÔ HÌNH ĐỘNG LỰC HỌC ---
    % I*omega_dot + omega×(I*omega) = tau
    omega_dot = I \ (tau - cross(omega, I*omega));

    % Tích phân cập nhật trạng thái
    omega = omega + omega_dot * dt;
    angles = angles + omega * dt; % xấp xỉ góc Euler (đủ chính xác cho góc nhỏ)

    % --- Lưu dữ liệu ---
    angles_hist(:,k) = angles;
    omega_hist(:,k)  = omega;
    tau_hist(:,k)    = tau;
    angle_sp_hist(:,k) = sp;
end

%% ==== VẼ ĐỒ THỊ KẾT QUẢ ====
figure('Name','Yaw + Attitude PID Demo','Position',[100 100 1100 700]);
subplot(3,2,1);
plot(t, rad2deg(angles_hist(1,:)),'b','LineWidth',1.2); hold on;
plot(t, rad2deg(angle_sp_hist(1,:)),'r--');
ylabel('Roll (deg)'); legend('roll','roll sp'); grid on;

subplot(3,2,3);
plot(t, rad2deg(angles_hist(2,:)),'b','LineWidth',1.2); hold on;
plot(t, rad2deg(angle_sp_hist(2,:)),'r--');
ylabel('Pitch (deg)'); legend('pitch','pitch sp'); grid on;

subplot(3,2,5);
plot(t, rad2deg(angles_hist(3,:)),'b','LineWidth',1.2); hold on;
plot(t, rad2deg(angle_sp_hist(3,:)),'r--');
ylabel('Yaw (deg)'); legend('yaw','yaw sp');
xlabel('Time (s)'); grid on;

subplot(3,2,2);
plot(t, omega_hist(1,:)); ylabel('p (rad/s)'); grid on;
subplot(3,2,4);
plot(t, omega_hist(2,:)); ylabel('q (rad/s)'); grid on;
subplot(3,2,6);
plot(t, omega_hist(3,:)); ylabel('r (rad/s)'); xlabel('Time (s)'); grid on;

%% ==== VẼ BIỂU ĐỒ MÔ-MEN ĐIỀU KHIỂN ====
figure('Name','Control Torques','Position',[200 200 700 300]);
plot(t, tau_hist(1,:),'r','LineWidth',1.2); hold on;
plot(t, tau_hist(2,:),'g','LineWidth',1.2);
plot(t, tau_hist(3,:),'b','LineWidth',1.2);
legend('tau_{roll}','tau_{pitch}','tau_{yaw}');
xlabel('Time (s)'); ylabel('Torque (N*m)'); grid on;

%% ==== MÔ PHỎNG 3D DRONE ====
figure('Name','3D Attitude Animation','Color','w');
axis equal; view(45,20);
axis_lim = 0.3;
axis([-axis_lim axis_lim -axis_lim axis_lim -axis_lim axis_lim]);
hold on;
xlabel('X'); ylabel('Y'); zlabel('Z');

% Vẽ thân drone
hBody = patch('Faces',bodyF,'Vertices',bodyV,'FaceColor',[0.7 0.7 0.8],'EdgeColor','k','FaceAlpha',0.9);
% Tay drone
armX = [ -armLen 0 armLen; 0 0 0; 0 0 0 ];
armY = [ 0 0 0; -armLen 0 armLen; 0 0 0 ];
hArm1 = plot3(armX(1,:),armX(2,:),armX(3,:),'k-','LineWidth',3);
hArm2 = plot3(armY(1,:),armY(2,:),armY(3,:),'k-','LineWidth',3);

% Mũi tên biểu diễn lực đẩy (thrust vector)
hThrust = quiver3(0,0,0,0,0,0,'b','LineWidth',2,'MaxHeadSize',0.6);

% Cập nhật theo thời gian
for k = 1:20:N
    phi = angles_hist(1,k);   % roll
    th  = angles_hist(2,k);   % pitch
    psi = angles_hist(3,k);   % yaw
    
    % Ma trận quay từ body → world
    R = rotz(rad2deg(psi)) * roty(rad2deg(th)) * rotx(rad2deg(phi));
    
    % Quay thân và cánh drone theo góc hiện tại
    V = (R * bodyV')';
    set(hBody,'Vertices',V);
    armXr = (R * armX)';
    armYr = (R * armY)';
    set(hArm1,'XData',armXr(:,1),'YData',armXr(:,2),'ZData',armXr(:,3));
    set(hArm2,'XData',armYr(:,1),'YData',armYr(:,2),'ZData',armYr(:,3));
    
    % Vẽ vector lực đẩy hướng theo trục Z của body
    thrustVec = R * [0;0;0.15];
    set(hThrust,'UData',thrustVec(1),'VData',thrustVec(2),'WData',thrustVec(3));
    
    title(sprintf('t = %.2f s  | Roll=%.1f°  Pitch=%.1f°  Yaw=%.1f°', ...
        t(k), rad2deg(phi), rad2deg(th), rad2deg(psi)));
    drawnow;
    pause(0.01);
end

%% ==== HÀM PHỤ ====
function [V,F] = makeBox(L,W,H)
    % Tạo hình hộp chữ nhật (body drone)
    x = L/2; y = W/2; z = H/2;
    V = [-x -y -z; x -y -z; x y -z; -x y -z;
         -x -y z;  x -y z;  x y z;  -x y z];
    F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
end

function R = rotx(a)
    % Ma trận quay quanh trục X (độ)
    a = deg2rad(a);
    R = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];
end
function R = roty(a)
    % Ma trận quay quanh trục Y (độ)
    a = deg2rad(a);
    R = [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)];
end
function R = rotz(a)
    % Ma trận quay quanh trục Z (độ)
    a = deg2rad(a);
    R = [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];
end
