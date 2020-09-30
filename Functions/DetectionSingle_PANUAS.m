function [detection] = DetectionSingle_PANUAS(scenario)
%DETECTIONSINGLE_PANUAS Performs target detection for PANUAS project
%   Takes scenario object as input, provides scenario.detection object as
%   output, containing information about detected targets.

%% Unpack Variables

detection = scenario.detection;
radarsetup = scenario.radarsetup;
cube = scenario.cube;
flags = scenario.flags;

%% Perform Detection

% Sum across angle slices
rd_cube = sum(cube.pow_cube, [3 4]);

% Estimate noise power
detection.noise_pow = pow2db(median(mean(rd_cube), 'all'));

switch radarsetup.detect_type
    case 'threshold'
        %% Perform Threshold Detection
        
        % Calculate threshold in absolute
        abs_thresh = db2pow(radarsetup.thresh + detection.noise_pow);
        
        % Perform detection
        detection.detect_cube = (rd_cube > abs_thresh);
        
    case 'CFAR'
        %% Perform CFAR Detection
        
        % Set up index map
        [n_rng, n_dop] = size(rd_cube);
        pad = radarsetup.num_guard + radarsetup.num_train;
        rng_ax = (pad(1) + 1):(n_rng-pad(1));
        dop_ax = (pad(2) + 1):(n_dop-pad(2));
        
        idx = [];
        idx(1,:) = repmat(rng_ax, 1, length(dop_ax));
        idx(2,:) = reshape(repmat(dop_ax, length(rng_ax), 1), 1, []);
        
        % Perform CFAR detection
        cfar_out = scenario.sim.CFAR(rd_cube, idx);
        
        % Reshape to radar cube size
        cfar_out = reshape(cfar_out, length(rng_ax), length(dop_ax));
        cfar_out = [zeros(pad(1), length(dop_ax)); cfar_out; zeros(pad(1), length(dop_ax))];
        cfar_out = [zeros(n_rng, pad(2)), cfar_out, zeros(n_rng, pad(2))];
        
        % Perform image dilation
        if radarsetup.dilate
            se = strel('disk', 1);
            cfar_out = imdilate(cfar_out, se);
        end
        
        % Save detection cube
        detection.detect_cube = cfar_out;
        
end

%% Update Multiple CPI List

% Initialize multi-frame arrays if not created
if isempty(detection.detect_cube_multi)
    detection.detect_cube_multi = zeros(size(detection.detect_cube));
    detection.pow_cube_multi = zeros(size(cube.pow_cube));
end

% Add to number of detections per cell
detection.detect_cube_multi = detection.detect_cube_multi + detection.detect_cube;

% Add to power cube
detection.pow_cube_multi = detection.pow_cube_multi + cube.pow_cube .* detection.detect_cube;

end



