classdef BatteryModel < handle
    % BatteryModel - Li-ion battery model for drone simulation
    % Works both in MATLAB scripts and Simulink (via MATLAB Function block).
    % Simulates voltage and SOC given current draw.
    
    properties
        Capacity_Ah = 2.2;          % Battery capacity (Ah)
        NominalVoltage = 11.1;      % Nominal voltage (V)
        InternalResistance = 0.05;  % Internal resistance (Ohm)
        SOC = 1.0;                  % State of charge
        dt = 0.01;                  % Simulation time step (s)  
        lastTime = 0;
    end

    
    methods
        function obj = BatteryModel(varargin)
            % Allow name-value initialization
            if nargin > 0
                for i = 1:2:length(varargin)
                    if isprop(obj, varargin{i})
                        obj.(varargin{i}) = varargin{i+1};
                    end
                end
            end
        end

        % === Step update (for Simulink) ===
        function [voltage, soc] = step(obj, current)
            % One simulation step (used inside MATLAB Function block)
            dQ = (current * obj.dt) / (obj.Capacity_Ah * 3600);
            obj.SOC = max(obj.SOC - dQ, 0);
            voltage = obj.NominalVoltage ...
                      - current * obj.InternalResistance ...
                      - (1 - obj.SOC) * 0.8;  % sag when low battery
            soc = obj.SOC;
        end
        
        % === Full offline simulation (for MATLAB) ===
        function [t, voltage, soc] = simulate(obj, T, currentProfile)
            % Simulate discharge for duration T with given current profile
            t = 0:obj.dt:T;
            n = length(t);
            
            if isscalar(currentProfile)
                I = currentProfile * ones(1, n);
            elseif length(currentProfile) == n
                I = currentProfile;
            else
                error('currentProfile must be scalar or same length as simulation time.');
            end
            
            soc = zeros(1, n);
            voltage = zeros(1, n);
            soc(1) = obj.SOC;
            
            for k = 2:n
                [voltage(k), soc(k)] = obj.step(I(k-1));
            end
        end
        
        % === Plot results ===
        function plotResults(~, t, voltage, soc)
            figure('Name','🔋 Battery Simulation','Color','w');
            tiledlayout(2,1);

            nexttile;
            plot(t, voltage, 'b', 'LineWidth', 1.6);
            grid on; ylabel('Voltage (V)');
            title('Battery Voltage vs Time');

            nexttile;
            plot(t, soc*100, 'r', 'LineWidth', 1.6);
            grid on; xlabel('Time (s)'); ylabel('State of Charge (%)');
        end
    end
end
