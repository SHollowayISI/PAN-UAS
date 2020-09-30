function [multi] = Tracking_PANUAS(scenario)
%TRACKING_PANUAS Multi-Target Tracking system for PAN-UAS project
%   Takes scenario object as input, returns modified multi object as child of
%   scenario.


%% Unpack Variables

radarsetup = scenario.radarsetup;
frame = scenario.flags.frame;
multi = scenario.multi;

%% Target-to-Track Association

% Initialize variables
detect_ind_list = [];
hit_list = [];

% Loop through all pre-existing tracks
for tr = multi.active_tracks
    
    % Infinite distance if no detection found
    dist = Inf;
    detect_ind = [];
    
    % Find detections in track uncertainty area
    for de = 1:multi.detect_list{frame}.num_detect
        
        % Calculate Mahanalobis distance
        dist_check = MahanalobisDistance( ...
            multi.detect_list{frame}, de, multi.track_list{tr}, ...
            radarsetup.tracking.EKF, radarsetup.tracking.sigma_z);
        
        % If detection is within bound and closer than previous
        if (dist_check < dist) && (dist_check < radarsetup.tracking.dist_thresh)
            
            % Save new distance
            dist = dist_check;
            detect_ind = de;
            
            % Assign detection coordinates
            multi.track_list{tr}.meas.range = multi.detect_list{frame}.range(de);
            multi.track_list{tr}.meas.az = multi.detect_list{frame}.az(de);
            multi.track_list{tr}.meas.el = multi.detect_list{frame}.el(de);
            multi.track_list{tr}.meas.vel = multi.detect_list{frame}.vel(de);
            multi.track_list{tr}.meas.cart = multi.detect_list{frame}.cart(:,de);
            
        end
    end
    
    % Check if a detection was found
    if isinf(dist)  % Non-detection case
        
        % Increment counter of misses if no detection
        multi.track_list{tr}.misses = multi.track_list{tr}.misses + 1;
        
        % Deactivate track if too many misses
        if multi.track_list{tr}.misses > radarsetup.tracking.miss_max
            
            % Remove from active track
            multi.active_tracks(multi.active_tracks == tr) = [];
            
        end
    else            % Detection case
        
        % Update timestep
        time_step(tr) = multi.track_list{tr}.misses + 1;
        
        % Set counter of misses to zero, increment number of hits
        multi.track_list{tr}.misses = 0;
        multi.track_list{tr}.hits = multi.track_list{tr}.hits + 1;
        
        % Append detection to detection list
        multi.track_list{tr}.det_list(:,end+1) = multi.track_list{tr}.meas.cart;
        
        % Add detection index to list
        detect_ind_list(end+1) = detect_ind;
        
        % Add hit index to list
        hit_list(end+1) = tr;
        
    end
    
    % Set false alarm flag if too few hits
    multi.track_list{tr}.false_alarm = ...
        (multi.track_list{tr}.hits < radarsetup.tracking.max_hits_fa);
        
end

% Create list of non-associated indices
ind = 1:multi.detect_list{frame}.num_detect;
ind(detect_ind_list) = [];

% Loop through non-associated detections
for de = ind
    
    % Pass coordinates from detection list
    meas = struct( ...
        'range',      multi.detect_list{frame}.range(de), ...
        'az',         multi.detect_list{frame}.az(de), ...
        'el',         multi.detect_list{frame}.el(de), ...
        'vel',        multi.detect_list{frame}.vel(de), ...
        'cart',       multi.detect_list{frame}.cart(:,de));
    
    % Construct kinematic uncertainty
    kin_pre = [meas.cart'; zeros(1,3)];
    kin_pre = kin_pre(:);
    
    % Create track with new detection
    multi.track_list{end+1} = struct( ...
        'hits',               1, ...
        'misses',             0, ...
        'false_alarm',        true, ...
        'kin_pre',            kin_pre, ...
        'unc_pre',            zeros(6), ...
        'meas',               meas, ...
        'det_list',           meas.cart, ...
        'est_list',           []);
    
    % Update track lists
    multi.active_tracks(end+1) = length(multi.track_list);
    hit_list(end+1) = length(multi.track_list);
    time_step(length(multi.track_list)) = 1;
    
    
end

%{

tracking contains multi.track_list and active_list

tracking.multi.track_list{n} fields:
    hits
    misses
    false_alarm
    kinematic_prediction
    kinematic_uncertainty
    measurement_coords
    estimate_list
    
Required parameters:
    max_misses
    min_hits_fa
    
%}

%{
TODO

1. Loop through existing tracks
foreach tracking.multi.track_list{active_list}
    SWITCH Detections in area:
        CASE 0
            miss++
            if miss > max_miss
                track.state = end
                if length == 1
                    track.false_alarm = true
        CASE 1
            track.measure_coords = detect.coords
            detect.associated = true
            miss = 0
        CASE >2
            track.measure_coords = argmin(dist(detect.coords)).coords
            detect.associated = true
            miss = 0

2. Loop through unassociated detections
    foreach detection in detect_list{frame}
        multi.track_list{end+1} = new track

%}

%% Kalman Filtering

%{
TODO

Pass in:
    Measurement position
    Kinematic prediction
    Uncertainty prediction
    Time step size

Return out:
    Kinematic estimate
    Kinematic prediction
    Uncertainty prediction

Required parameters:
    sigma_z [3]
    sigma_v [3]
    smooth  [3] (???)

- Compute Kalman estimation for every active track
    - Pass in correct time step for missed tracks
    - Pass in type of Kalman filter?
- Save estimates to kalman_uncertainty, kalman_prediction, and
estimate_list

%}

% Loop through active tracks and update Kalman tracking
for tr = hit_list
    
    % Unpack current track
    curr_tr = multi.track_list{tr};
    
    % Calculate timestep
    kalman_Tm = radarsetup.t_fr * time_step(tr);
    
    % Run single step of Kalman predictor-corrector algorithm
    [kin_est, kin_pre, unc_pre] = KalmanFilter_Step( ...
        curr_tr.meas, curr_tr.kin_pre, curr_tr.unc_pre, kalman_Tm, ...
        radarsetup.tracking.sigma_v, radarsetup.tracking.sigma_z, ...
        radarsetup.tracking.EKF);
    
    % Re-pack to current track
    curr_tr.est_list(:,end+1) = kin_est;
    curr_tr.kin_pre = kin_pre;
    curr_tr.unc_pre = unc_pre;
    multi.track_list{tr} = curr_tr;
    
    
end


end

