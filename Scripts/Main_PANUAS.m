%% PANUAS Radar System - Main Simulation Loop
%{

    Sean Holloway
    PANUAS Main Simulation Loop
    
    This file specifies performs simulation, signal processing, detection,
    data processing, and results collection for PANUAS system.

    Use script 'FullSystem_PANUAS.m' to run scenarios.
    
%}

%% Main Loop

% Start timing for estimation
timeStart(scenario);

for loop = 1:scenario.simsetup.num_frames
    
    scenario.flags.frame = loop;
    
    for cpi = 1:scenario.radarsetup.cpi_fr
        
        scenario.flags.cpi = cpi;
        
        %% Radar Simulation
        
        % Run simulation to retrieve fast time x slow time x rx-channel signal
        scenario = RadarSimulation_PANUAS(scenario);
        
        %% Signal Processing
        
        % Perform signal processing on received signal
        scenario.cube = SignalProcessing_PANUAS(scenario);
        
        % Generate 3-D Cartesian Mesh Grids
        generateCoordinates(scenario);
        
        %% Single CPI Data Processing
        
        % Perform single-frame radar detection
        scenario.detection = DetectionSingle_PANUAS(scenario);
        
        %% Loop Update Procedures
        
        % Read out CPI update
        CPIUpdate(scenario);
        
        % Read out estimated time of completion
        timeUpdate(scenario, 1, 'loops')
        
    end
    
    %% Multiple CPI Data Processing
    
    % Perform binary integration and coordinate determination
    scenario.detection = DetectionMultiple_PANUAS(scenario);
    
    % Read out detection data
    if scenario.simsetup.readout
        readOut(scenario);
    end
    
    % Save detection data
    saveMulti(scenario);
    
    % Update multi-target tracking system
    scenario.multi = Tracking_PANUAS(scenario);
    
    %% Single Slice Visualization
    
    %{
    % View Range-Doppler heat map of center azimuth-elevation direction
    % viewRDCube(scenario, 'heatmap')
    
    % View Range-Doppler surface of center azimuth-elevation direction
    % viewRDCube(scenario, 'surface')
    
    % View Range-Angle heat map of zero-doppler slice
    % viewRACube(scenario, 'heatmap')
    
    % View Range-Angle surface of zero-doppler slice
    % viewRACube(scenario, 'surface')
    
    % View Range-Angle PPI of zero-doppler slice
    % viewRACube(scenario, 'PPI')
    
    % View single-frame detection heatmap
    % viewDetections(scenario, 'heatmap')
    
    % View single-frame detection PPI
    % viewDetections(scenario, 'PPI')
    %}
    
end

%% Data Visualization

% Display detections in 3D scatter plot
% viewDetections3D(scenario);

% Display result visualization plots
% viewTracking(scenario);








