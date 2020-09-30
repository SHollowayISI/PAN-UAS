%% PANUAS Radar System - Example Radar Initialization File
%{

    Sean Holloway
    PANUAS Init File
    
    This file specifies radar parameters for PANUAS simulation.

    Use script 'FullSystem_PANUAS.m' to run scenarios.
    
%}

%% Radar Parameter Setup

% Radar simulation and processing setup
scenario.radarsetup = struct( ...
    ...
    ... % Waveform Properties
    'f_c',          5.8e9, ...              % Operating frequency in Hz
    'f_s',          51.2e6, ...             % ADC sample frequency in Hz
    't_ch',         50e-6, ...              % Chirp duration in seconds
    'bw',           200e6, ...              % Chirp bandwidth in Hz
    'n_p',          512, ...                % Number of (MIMO) chirps per CPI
    'drop_s',       512, ...                % Number of samples to drop
    'cpi_fr',       10, ...                 % Number of CPI per frame
    'mimo_type',    'CDM', ...              % Set 'TDM' or 'CDM'
    'angle_method', 'set', ...              % Set 'fit' or 'set'
    'n_el',         16, ...                 % Size of elevation FFT for 'set' case
    'n_az',         16, ...                 % Size of azimuth FFT for 'set' case
    ...
    ... % Antenna Array Properties
    'n_tx_y',       2, ...              % Number of horizontal elements in Tx array
    'n_tx_z',       2, ...              % Number of vertical elements in Tx array
    'd_tx',         2, ...              % Distance between Tx elements in wavelengths
    'n_rx_y',       4, ...              % Number of horizontal elements in Rx array
    'n_rx_z',       4, ...              % Number of vertical elements in Rx array
    'd_rx',         0.5, ...            % Distance between Rx elements in wavelengths
    ...
    ... % Transceiver Properties
    'tx_pow',       db2pow(-1), ...     % Transmit power per antenna in Watts
    'rf_sys_loss',  0, ...              % RF system loss in dB
    'rx_nf',        1.8, ...            % Rx noise figure in dB
    ...
    ... % Antenna Properties
    'ant_gain',     db2pow(6.88), ...   % Antenna gain in absolute
    'ant_cos_pow',  [1 1], ...          % Cosine power of antenna pattern
    ...
    ... % Processing Properties
    'r_win',        'hanning', ...          % Window for range processing
    'd_win',        'blackmanharris', ...   % Window for doppler processing
    'az_win',       'hanning', ...          % Window for azimuth processing
    'el_win',       'hanning', ...          % Window for elevation processing
    ...
    ... % Detection Properties
    'detect_type',  'CFAR', ...         % Choose 'CFAR' or 'threshold'
    'CFAR_Pfa',     0.1, ...            % CFAR false alarm probability
    'num_guard',    [5 5], ...          % Number of R-D guard cells for CFAR detection
    'num_train',    [25 25], ...        % Number of R-D training cells for CFAR detection
    'dilate',       true, ...           % T/F dilate raw CFAR result to avoid duplicates   
    'det_m',        2);                 % M for m-of-n binary integration

tracking = struct( ...
    ...
    ... % Tracking properties
    'dist_thresh',  Inf, ...            % Mahanalobis distance association threshold
    'miss_max',     2, ...            % Number of misses required to inactivate track
    'max_hits_fa',  1, ...              % Maximum number of hits for track to still be false alarm
    'EKF',          false, ...           % T/F use extended Kalman filter
    'sigma_v',      [4.5 4.5 4.5], ...        % XYZ target motion uncertainty
    'sigma_z',      [0.5 0.5 0.5 1]);         % XYZnull or RAEV measurement uncertainty

scenario.radarsetup.tracking = tracking;


%% Run Setup Scripts

% Set up Phased Array Toolbox system objects
scenario = PhasedSetup_PANUAS(scenario);





