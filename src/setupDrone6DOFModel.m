function setupDrone6DOFModel(baseModel)
    %=== Helper to safely add a line ===%
    function safe_add_line(sys, src, dst)
        % Avoid duplicate or conflicting connections
        lines = get_param(sys, 'Lines');
        for i = 1:numel(lines)
            l = lines(i);
            try
                if strcmp([l.SrcBlock '/' num2str(l.SrcPort)], src) && ...
                   strcmp([l.DstBlock '/' num2str(l.DstPort)], dst)
                    return; % already connected
                end
            end
        end
        % Delete destination line if already used
        try, delete_line(sys, dst); catch, end
        % Try add, handle errors gracefully
        try
            add_line(sys, src, dst, 'autorouting', 'on');
        catch err
            warning("safe_add_line: could not add %s -> %s in %s\nReason: %s", src, dst, sys, err.message);
        end
    end

    %% --- Input defaults ---
    if nargin < 1
        baseModel = 'models/quadcopter_simple';
    end

    [basePath, baseName, ~] = fileparts(baseModel);
    load_system(baseModel);
    disp(['Loaded base model: ', baseModel]);

    %% --- Create new model ---
    newModel = [baseName, '_6DOF'];
    newModelPath = fullfile(basePath, [newModel, '.slx']);
    if bdIsLoaded(newModel)
        close_system(newModel, 0);
    end
    new_system(newModel);
    open_system(newModel);
    disp('Created new top-level model.');

    %% --- Subsystems ---
    subsystems = {'Controller','Mixer','Motors','Plant6DOF','Sensors','Battery'};
    x0 = 100; y0 = 100; dy = 120;
    for i = 1:numel(subsystems)
        try
            add_block('built-in/Subsystem', [newModel '/' subsystems{i}], ...
                'Position', [x0 y0+(i-1)*dy x0+200 y0+60+(i-1)*dy]);
        catch
        end
    end

    %% --- Dummy ports ---
    for i = 1:numel(subsystems)
        sysPath = [newModel '/' subsystems{i}];
        if strcmp(subsystems{i}, 'Controller'), continue; end
        open_system(sysPath);
        blkList = get_param(sysPath, 'Blocks');
        if ~any(strcmp(blkList, 'In1'))
            add_block('built-in/Inport', [sysPath '/In1'], 'Position', [30 40 60 60]);
        end
        if ~any(strcmp(blkList, 'Out1'))
            add_block('built-in/Outport', [sysPath '/Out1'], 'Position', [230 40 260 60]);
        end
        safe_add_line(sysPath, 'In1/1', 'Out1/1');
    end

    %% --- Controller ---
    ctrlPath = [newModel '/Controller']; open_system(ctrlPath);
    try delete_block([ctrlPath '/In1']); end
    try delete_block([ctrlPath '/Out1']); end
    add_block('built-in/Inport', [ctrlPath '/In1'], 'Position', [30 110 60 130]);
    add_block('built-in/Outport', [ctrlPath '/Out1'], 'Position', [400 140 430 160]);
    add_block('built-in/Subsystem', [ctrlPath '/PositionControl'], 'Position', [150 60 290 120]);
    add_block('built-in/Subsystem', [ctrlPath '/AttitudeControl'], 'Position', [150 160 290 220]);

    % PositionControl internals
    posPath = [ctrlPath '/PositionControl']; open_system(posPath);
    add_block('built-in/Inport', [posPath '/In1'], 'Position', [30 40 60 60]);
    add_block('built-in/Outport', [posPath '/Out1'], 'Position', [230 40 260 60]);
    add_block('simulink/Continuous/PID Controller', [posPath '/PID_X'], ...
        'P','1','I','0.5','D','0.05','Position',[90 30 180 70]);
    safe_add_line(posPath, 'In1/1', 'PID_X/1');
    safe_add_line(posPath, 'PID_X/1', 'Out1/1');

    % AttitudeControl internals
    attPath = [ctrlPath '/AttitudeControl']; open_system(attPath);
    add_block('built-in/Inport', [attPath '/In1'], 'Position', [30 40 60 60]);
    add_block('built-in/Outport', [attPath '/Out1'], 'Position', [230 40 260 60]);
    add_block('simulink/Continuous/PID Controller', [attPath '/PID_Roll'], ...
        'P','1','I','0.4','D','0.02','Position',[90 30 180 70]);
    safe_add_line(attPath, 'In1/1', 'PID_Roll/1');
    safe_add_line(attPath, 'PID_Roll/1', 'Out1/1');

    % Controller wiring
    safe_add_line(ctrlPath, 'In1/1', 'PositionControl/1');
    safe_add_line(ctrlPath, 'PositionControl/1', 'AttitudeControl/1');
    safe_add_line(ctrlPath, 'AttitudeControl/1', 'Out1/1');

    %% --- Mixer ---
    mixPath = [newModel '/Mixer']; open_system(mixPath);
    add_block('simulink/Math Operations/Gain', [mixPath '/Gain'], ...
        'Gain', '[1 1 1 1]', 'Position', [100 40 180 80]);
    safe_add_line(mixPath, 'In1/1', 'Gain/1');
    safe_add_line(mixPath, 'Gain/1', 'Out1/1');

    %% --- Motors ---
    motPath = [newModel '/Motors']; open_system(motPath);
    add_block('simulink/Continuous/Transfer Fcn', [motPath '/MotorDyn'], ...
        'Numerator', '[1]', 'Denominator', '[0.05 1]', 'Position', [90 40 170 80]);
    safe_add_line(motPath, 'In1/1', 'MotorDyn/1');
    safe_add_line(motPath, 'MotorDyn/1', 'Out1/1');

%% === Plant6DOF (Simplified 6DOF Dynamics) ===
plantPath = [newModel '/Plant6DOF']; open_system(plantPath);

% Check and create blocks if missing
if ~any(strcmp(get_param(plantPath,'Blocks'),'Sum'))
    add_block('simulink/Math Operations/Sum', [plantPath '/Sum'], ...
        'Inputs', '|+-', 'Position', [120 50 150 90]);
end
if ~any(strcmp(get_param(plantPath,'Blocks'),'Gain'))
    add_block('simulink/Math Operations/Gain', [plantPath '/Gain'], ...
        'Gain', '0.98', 'Position', [180 50 230 90]);
end
if ~any(strcmp(get_param(plantPath,'Blocks'),'Integrator'))
    add_block('simulink/Continuous/Integrator', [plantPath '/Integrator'], ...
        'Position', [250 50 290 90]);
end

% Wiring
safe_add_line(plantPath, 'In1/1', 'Sum/1');
safe_add_line(plantPath, 'Sum/1', 'Gain/1');
safe_add_line(plantPath, 'Gain/1', 'Integrator/1');
safe_add_line(plantPath, 'Integrator/1', 'Out1/1');


    %% --- Sensors ---
    senPath = [newModel '/Sensors']; open_system(senPath);
    add_block('simulink/Sources/Random Number', [senPath '/Noise'], ...
        'Mean','0','Variance','0.01','Position',[90 30 150 70]);
    add_block('simulink/Math Operations/Sum', [senPath '/Sum'], ...
        'Inputs','+-','Position',[170 35 200 65]);
    safe_add_line(senPath, 'In1/1', 'Sum/1');
    safe_add_line(senPath, 'Noise/1', 'Sum/2');
    safe_add_line(senPath, 'Sum/1', 'Out1/1');

    %% --- Battery ---
    batPath = [newModel '/Battery']; open_system(batPath);
    add_block('simulink/Sources/Constant', [batPath '/Vbat'], ...
        'Value','11.1','Position',[80 40 130 70]);
    safe_add_line(batPath, 'Vbat/1', 'Out1/1');

    %% --- Top-level I/O and wiring ---
    add_block('built-in/Inport', [newModel '/x_ref'], 'Position', [30 80 60 100]);
    add_block('built-in/Outport', [newModel '/x_out'], 'Position', [900 80 930 100]);
    safe_add_line(newModel, 'x_ref/1', 'Controller/1');
    safe_add_line(newModel, 'Controller/1', 'Mixer/1');
    safe_add_line(newModel, 'Mixer/1', 'Motors/1');
    safe_add_line(newModel, 'Motors/1', 'Plant6DOF/1');
    safe_add_line(newModel, 'Plant6DOF/1', 'Sensors/1');
    safe_add_line(newModel, 'Sensors/1', 'x_out/1');

    %% --- Save and update ---
    set_param(newModel, 'SimulationCommand', 'update');
    save_system(newModel, newModelPath);
    open_system(newModel);
    disp(['✅ Created upgraded model: ', newModelPath]);
end
