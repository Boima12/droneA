classdef BatteryModel
    % BatteryModel - Simple Li-ion battery simulation for drone
    % This model provides a basic voltage drop and SOC (state-of-charge) simulation.
    
    properties
        Capacity_Ah = 2.2;      % Battery capacity (Ah)
        NominalVoltage = 11.1;  % Nominal voltage (V)
        InternalResistance = 0.05; % Ohm
        SOC = 1.0;              % Initial State of Charge (1 = 100%)
        dt = 0.01;              % Time step
    end
    
    methods
        function obj = BatteryModel(varargin)
            % Allow passing in parameters
            if nargin > 0
                for i = 1:2:length(varargin)
                    if isprop(obj, varargin{i})
                        obj.(varargin{i}) = varargin{i+1};
                    end
                end
            end
        end
        
        function [t, voltage, soc] = simulate(obj, T, currentProfile)
            % Simulate battery discharge under current profile
            % Inputs:
            %   T - total simulation time (s)
            %   currentProfile - current draw (A), can be scalar or vector
            
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
                dQ = (I(k-1) * obj.dt) / (obj.Capacity_Ah * 3600);
                soc(k) = max(soc(k-1) - dQ, 0);
                voltage(k) = obj.NominalVoltage - I(k-1)*obj.InternalResistance ...
                    - (1 - soc(k)) * 0.8; % 0.8V sag when near empty
            end
        end
        
        function plotResults(~, t, voltage, soc)
            figure('Name','Battery Simulation');
            subplot(2,1,1);
            plot(t, voltage, 'LineWidth', 1.5); grid on;
            ylabel('Voltage (V)');
            title('Battery Voltage vs Time');
            
            subplot(2,1,2);
            plot(t, soc*100, 'LineWidth', 1.5);
            grid on; ylabel('State of Charge (%)'); xlabel('Time (s)');
        end
    end
end
