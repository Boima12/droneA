classdef YawAttitudePID
    % YawAttitudePID - nested PID controller for roll, pitch, yaw attitude control
    % Simulates outer-loop (angle) and inner-loop (rate) PID controllers
    % for a quadrotor attitude control subsystem.
    
    properties
        dt = 0.001;         % Time step (s)
        Tsim = 8;           % Simulation duration (s)
        I = diag([0.02, 0.025, 0.03]); % Inertia matrix (kg*m^2)
        
        % Outer-loop (angle) PID gains
        Kp_ang = [6, 6, 4];
        Ki_ang = [0.5, 0.5, 0.2];
        Kd_ang = [1.0, 1.0, 0.6];
        
        % Inner-loop (rate) PID gains
        Kp_rate = [0.12, 0.12, 0.08];
        Ki_rate = [0.01, 0.01, 0.005];
        Kd_rate = [0.002, 0.002, 0.001];
        
        % Limits
        rate_sp_max = deg2rad([200, 200, 120]);
        torque_limit = [0.15, 0.15, 0.15];
        
        % Setpoints (deg)
        roll_sp = 10;
        pitch_sp = -5;
        yaw_sp = 30;
        
        % Initial conditions
        omega0 = [0; 0; 0];
        angles0 = [0; 0; 0];
    end
    
    methods
        function obj = YawAttitudePID(varargin)
            % Allow overriding parameters
            for i = 1:2:length(varargin)
                if isprop(obj, varargin{i})
                    obj.(varargin{i}) = varargin{i+1};
                end
            end
        end
        
        function results = simulate(obj)
            % Simulate nested PID loops
            
            t = 0:obj.dt:obj.Tsim;
            N = length(t);
            I = obj.I;
            
            % State variables
            omega = obj.omega0;
            angles = obj.angles0;
            int_ang = zeros(3,1);
            prev_err_ang = zeros(3,1);
            int_rate = zeros(3,1);
            prev_err_rate = zeros(3,1);
            
            % Disturbance
            disturbance = @(ti) (ti>3 && ti<3.5).* [0.02; -0.015; 0.03];
            
            % Histories
            angles_hist = zeros(3,N);
            omega_hist = zeros(3,N);
            tau_hist = zeros(3,N);
            sp_hist = zeros(3,N);
            
            % === Main loop ===
            for k = 1:N
                ti = t(k);
                
                % Desired angles
                if ti < 1
                    sp = deg2rad([0; 0; 0]);
                else
                    sp = deg2rad([obj.roll_sp; obj.pitch_sp; obj.yaw_sp]);
                end
                
                % Outer loop: angle PID -> desired angular rate
                err_ang = sp - angles;
                int_ang = int_ang + err_ang * obj.dt;
                derr_ang = (err_ang - prev_err_ang) / obj.dt;
                rate_sp = obj.Kp_ang'.*err_ang + obj.Ki_ang'.*int_ang + obj.Kd_ang'.*derr_ang;
                rate_sp = max(min(rate_sp, obj.rate_sp_max'), -obj.rate_sp_max');
                prev_err_ang = err_ang;
                
                % Inner loop: rate PID -> control torque
                err_rate = rate_sp - omega;
                int_rate = int_rate + err_rate * obj.dt;
                derr_rate = (err_rate - prev_err_rate) / obj.dt;
                tau = obj.Kp_rate'.*err_rate + obj.Ki_rate'.*int_rate + obj.Kd_rate'.*derr_rate;
                prev_err_rate = err_rate;
                
                % Anti-windup + torque limit
                for i=1:3
                    if tau(i) > obj.torque_limit(i)
                        tau(i) = obj.torque_limit(i);
                        int_rate(i) = int_rate(i) - err_rate(i)*obj.dt;
                    elseif tau(i) < -obj.torque_limit(i)
                        tau(i) = -obj.torque_limit(i);
                        int_rate(i) = int_rate(i) - err_rate(i)*obj.dt;
                    end
                end
                
                % Disturbance
                tau = tau + disturbance(ti);
                
                % Dynamics
                omega_dot = I \ (tau - cross(omega, I*omega));
                omega = omega + omega_dot * obj.dt;
                angles = angles + omega * obj.dt;
                
                % Record
                angles_hist(:,k) = angles;
                omega_hist(:,k) = omega;
                tau_hist(:,k) = tau;
                sp_hist(:,k) = sp;
            end
            
            results.t = t;
            results.angles = angles_hist;
            results.omega = omega_hist;
            results.tau = tau_hist;
            results.sp = sp_hist;
            
            % Plot results
            obj.plotResults(results);
        end
        
        function plotResults(~, results)
            t = results.t;
            angles = results.angles;
            sp = results.sp;
            omega = results.omega;
            tau = results.tau;
            
            % === Angle tracking ===
            figure('Name','Yaw + Attitude PID','Position',[100 100 1100 700]);
            subplot(3,2,1);
            plot(t, rad2deg(angles(1,:)),'b','LineWidth',1.2); hold on;
            plot(t, rad2deg(sp(1,:)),'r--');
            ylabel('Roll (deg)'); legend('roll','ref'); grid on;
            
            subplot(3,2,3);
            plot(t, rad2deg(angles(2,:)),'b','LineWidth',1.2); hold on;
            plot(t, rad2deg(sp(2,:)),'r--');
            ylabel('Pitch (deg)'); legend('pitch','ref'); grid on;
            
            subplot(3,2,5);
            plot(t, rad2deg(angles(3,:)),'b','LineWidth',1.2); hold on;
            plot(t, rad2deg(sp(3,:)),'r--');
            ylabel('Yaw (deg)'); legend('yaw','ref'); xlabel('Time (s)'); grid on;
            
            subplot(3,2,2);
            plot(t, omega(1,:)); ylabel('p (rad/s)'); grid on;
            subplot(3,2,4);
            plot(t, omega(2,:)); ylabel('q (rad/s)'); grid on;
            subplot(3,2,6);
            plot(t, omega(3,:)); ylabel('r (rad/s)'); xlabel('Time (s)'); grid on;
            
            % === Torques ===
            figure('Name','Control Torques');
            plot(t, tau(1,:),'r', t, tau(2,:),'g', t, tau(3,:),'b','LineWidth',1.2);
            legend('τ_roll','τ_pitch','τ_yaw'); grid on;
            xlabel('Time (s)'); ylabel('Torque (N*m)');
        end
    end
end
