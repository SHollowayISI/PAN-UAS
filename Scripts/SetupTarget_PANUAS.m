%% PANUAS Radar System - Target Initialization File
%{

    Sean Holloway
    PANUAS Target Init File
    
    This file specifies the list of targets used in the PANUAS simulation
    system.

    Use script 'FullSystem_PANUAS.m' to run scenarios.
    
%}

%% Target Setup

% Target positions in meters

% r = r_in;
% az = az_in;
% el = el_in;

r = 950;
az = 0;
el = 0;
tgt_pos = [r*cosd(el)*cosd(az); r*cosd(el)*sind(az); r*sind(el)];
% tgt_pos = [50,     300,     900; ...
%           -50,     25,      0; ...
%            20,     25,      0];
       
% Target velocities in meters per second
% v = v_in;
v = 0;
tgt_vel = [v*cosd(el)*cosd(az); v*cosd(el)*sind(az); v*sind(el)];
% tgt_vel = [40,   0,    -20; ...
%            10,   0,      0; ...
%             0,  10,      0];

%% Set Target RCS

% Target RCS in dBm^2
% tgt_rcs_dbmm = [-20, -20, -20];
% tgt_rcs_dbmm = rcs_in;
tgt_rcs_dbmm = -20;

% Convert to absolute
tgt_rcs = db2pow(tgt_rcs_dbmm);

%% Save Target List Structure

scenario.target_list = struct( ...
    'pos',              tgt_pos, ...
    'vel',              tgt_vel, ...
    'rcs',              tgt_rcs);



