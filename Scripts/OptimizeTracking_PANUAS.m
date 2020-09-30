%% PANUAS Radar System - Tracker Optimization Script
%{

    Sean Holloway
    PANUAS Tracker Optimizer
    
    This script runs a coordinate descent minimization of Kalman Filter and
    Extended Kalman Filter for PAN-UAS
    
%}

%% Housekeeping
clear variables;
close all;

%% Set Optimization Parameters

% Set coordinate-parameter list
coord_params = {'sigma_v', 'sigma_z'};

% Set starting points
coord_start = [1 1];

% Set [max; min]
coord_lim = [1,  0.01; ...
    100,    2.5];

% Set optimizer parameters
num_loops = 3;                  % Number of times to examine each coordinate
num_points = [100, 100, 100];    % Number of points to check, per loop

%% Set Dataset Parameters

num_paths = 500;                % Number of trajectories to simulate per point
num_frames = 100;               % Number of frames per trajectory

T = 0.1024;                     % Time step between points
time = T*((1:num_frames) - 1);

speed_max = 50;                 % Maximum velocity
z_real = 0.25;                     % Measurement variance

%% Main Loops

% Initialize coordinate
coord_best = coord_start;

% Step through optimizer loops
for loop = 1:num_loops
    
    % Pass in best coordinate
    coord_current = coord_best;
    
    % Step through coordinates
    for coord = 1:length(coord_start)
        
        % Generate list of mesh points
        mesh = linspace(coord_lim(1,coord), coord_lim(2,coord), num_points(loop));
        
        % Step through mesh points
        for pnt = 1:length(mesh)
            
            % Pass in coordinates to function parameters
            coord_current(coord) = mesh(pnt);
            
            % Step through dataset
            for path = 1:num_paths
                
                % Generate path
                x_st = [0; 0; 0];
                phi = (rand(1) - 0.5)*360;
                theta = (rand(1) - 0.5)*180;
                speed = rand(1)*speed_max;
                v_st = speed * [cosd(theta)*cosd(phi); cosd(theta)*sind(phi); sind(theta)];
                traj = x_st + v_st .* time;
                
                % Generate measurements
                meas_in = traj + z_real .* randn(size(traj));
                
                % Set up initial track
                kin_pre = [meas_in(:,1)'; zeros(1,3)];
                kin_pre = kin_pre(:);
                unc_pre = zeros(6);
                
                % Step through Kalman Loop
                for n = 1:num_frames
                    
                    meas.cart = meas_in(:,n);
                    
                    % Run single step of Kalman predictor-corrector algorithm
                    [kin_est, kin_pre, unc_pre] = KalmanFilter_Step( ...
                        meas, kin_pre, unc_pre, T, ...
                        coord_current(1) * ones(3,1), coord_current(2) * ones(3,1), false);
                    est_list(:,n) = kin_est;
                    
                end
                
                % Estimate Error
                path_error(path) = rms(sqrt(sum((est_list([1 3 5],:) - traj).^2)));
                
            end
            
            % Calculate RMS error for point
            error_list(pnt) = rms(path_error);
            
        end
        
        % Calculate best point from list
        [best_error, ind] = min(error_list);
        
        % Update best coordinate choice
        coord_best(coord) = mesh(ind);
        
        % Update current coordinate choice
        coord_current(coord) = coord_best(coord);
        
    end
    
    % Read out update
    msg = sprintf('Loops Complete: %d', loop);
    disp(msg);
    
    % Read out results
    for n = 1:length(coord_best)
        
        msg = sprintf(': %f', coord_best(n));
        msg = [coord_params{n}, msg];
        disp(msg)
        
    end
    
    % Read out RMS error
    msg = sprintf('Min RMS error: %f', best_error);
    disp(msg)
    disp('')
    
end
























