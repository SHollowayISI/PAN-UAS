%% PAN-UAS Radar System
%{

    Sean Holloway
    PAN-UAS (Portable Anti-UAS System)
    MATLAB Simulation & Processing

    This shell file runs successive scripts and gauges progress.

    TODO:
    
        - Improve AoA estimation algorithm
        - Save micro-doppler information
        - Tune Manahalobis threshold

%}

%% Housekeeping
clear variables
close all
addpath(genpath(pwd));
tic

%% Initialize Scenario Object

% Initialization
scenario = RadarScenario_PANUAS;

%% Setup Structures for Simulation

% Set up simulation parameters
SetupSimulation_PANUAS

% Set up radar target
SetupTarget_PANUAS

% Set up transceiver and channel parameters
SetupRadarScenario_PANUAS

%% Run Simulation & Signal Processing

% Perform main processes of simulation, signal and data processing
Main_PANUAS

%% Save and Package Resultant Data

% Run all end-of-simulation tasks
EndSimulationSingle_PANUAS














