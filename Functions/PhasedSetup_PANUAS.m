function [scenario] = PhasedSetup_PANUAS(scenario_in)
%PHASEDSETUP_PANUAS Sets up Phased Array Toolbox system objects for PANUAS
%   Takes scenario object as input and outputs scenario.sim struct
%   containing system objects for Phased Array Toolbox simulation.

%% Unpack Variables

scenario = scenario_in;
simsetup = scenario.simsetup;
radarsetup = scenario.radarsetup;
flags = scenario.flags;
target_list = scenario.target_list;

%% Set up constants

c = physconst('LightSpeed');
lambda = c/radarsetup.f_c;

%% Perform Calculations

% Ensure integer number of samples per chirp
radarsetup.f_s = floor(radarsetup.t_ch*radarsetup.f_s)/radarsetup.t_ch;

% Calculate number of samples per chirp
radarsetup.n_s = radarsetup.f_s*radarsetup.t_ch;

% Calculate frame time
radarsetup.t_fr = radarsetup.t_ch * radarsetup.n_p * ...
    radarsetup.n_tx_y * radarsetup.n_tx_z;

%% Signal Setup

% Transmit waveform
sim.waveform = phased.FMCWWaveform( ...
    ...
    'SampleRate',               radarsetup.f_s, ...
    'SweepTime',                radarsetup.t_ch, ...
    'SweepBandwidth',           radarsetup.bw, ...
    'SweepDirection',           'Up', ...
    'SweepInterval',            'Symmetric', ...
    'OutputFormat',             'Sweeps', ...
    'NumSweeps',                simsetup.sim_rate);

%% Motion Platform Setup

% Set up transceiver platform
sim.radar_plat = phased.Platform( ...
    ...
    'MotionModel',              'Velocity', ...
    'InitialPosition',          simsetup.radar_pos, ...
    'Velocity',                 simsetup.radar_vel);

%% Target Setup

% Set up fluctuating target model
sim.target = phased.RadarTarget( ...
    'EnablePolarization',       false, ...
    'MeanRCSSource',            'Property', ...
    'MeanRCS',                  target_list.rcs, ...
    'Model',                    'Swerling4', ...
    'OperatingFrequency',       radarsetup.f_c, ...
    'PropagationSpeed',         c);

% Set up target platform position and velocity
sim.target_plat = phased.Platform( ...
    ...
    'MotionModel',              'Velocity', ...
    'InitialPosition',          target_list.pos, ...
    'Velocity',                 target_list.vel);

%% Channel Setup

% Set up channel model
sim.channel = phased.LOSChannel( ...
    'PropagationSpeed',         c, ...
    'OperatingFrequency',       radarsetup.f_c, ...
    'TwoWayPropagation',        true, ...
    'SampleRate',               radarsetup.f_s);

%% Antenna Setup

% Create antenna element object
sim.antenna = phased.CosineAntennaElement( ...
    'FrequencyRange',           radarsetup.f_c * [0.9 1.1], ...
    'CosinePower',              radarsetup.ant_cos_pow);

% Create Tx rectangular array
sim.tx_array = phased.URA( ...
    'Element',                  sim.antenna, ...
    'Size',                     [radarsetup.n_tx_z, radarsetup.n_tx_y], ...
    'ArrayNormal',              'x', ...
    'ElementSpacing',           lambda * [radarsetup.d_tx, radarsetup.d_tx]);

% Create Rx rectangular array
sim.rx_array = phased.URA( ...
    'Element',                  sim.antenna, ...
    'Size',                     [radarsetup.n_rx_z, radarsetup.n_rx_y], ...
    'ArrayNormal',              'x', ...
    'ElementSpacing',           lambda * [radarsetup.d_rx, radarsetup.d_rx]);

%% Transceiver Setup

% Set up transmitter parameters
sim.transmitter = phased.Transmitter( ...
    'PeakPower',                radarsetup.tx_pow, ...
    'Gain',                     pow2db(radarsetup.ant_gain), ...
    'LossFactor',               radarsetup.rf_sys_loss, ...
    'InUseOutputPort',          false);
    
% Set up radiation interface
sim.radiator = phased.Radiator( ...
    'Sensor',                   sim.tx_array, ...
    'PropagationSpeed',         c, ...
    'OperatingFrequency',       radarsetup.f_c, ...
    'CombineRadiatedSignals',   true, ...
    'WeightsInputPort',         true);

% Set up receiver parameters
sim.receiver = phased.ReceiverPreamp( ...
    'Gain',                     pow2db(radarsetup.ant_gain),  ...
    'NoiseFigure',              radarsetup.rx_nf, ...
    'SampleRate',               radarsetup.f_s);
    
% Set up receiver radiation interface
sim.collector = phased.Collector( ...
    'Sensor',                   sim.rx_array, ...
    'PropagationSpeed',         c, ...
    'OperatingFrequency',       radarsetup.f_c, ...
    'Wavefront',                'Plane');


%% Detection Setup

% Set up CFAR detector
sim.CFAR = phased.CFARDetector2D( ...
    'Method',                   'CA', ...
	'ProbabilityFalseAlarm',    radarsetup.CFAR_Pfa, ...
    'ThresholdFactor',          'Auto', ...
    'GuardBandSize',            radarsetup.num_guard, ...
    'TrainingBandSize',         radarsetup.num_train);


%% Re-pack Variables

scenario.sim = sim;
scenario.simsetup = simsetup;
scenario.radarsetup = radarsetup;
scenario.flags = flags;

end