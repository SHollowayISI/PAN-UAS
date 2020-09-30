function [detection] = DetectionMultiple_PANUAS(scenario)
%DETECTIONMULTIPLE_PANUAS Summary of this function goes here
%   Detailed explanation goes here

%% Unpack Variables

detection = scenario.detection;
radarsetup = scenario.radarsetup;
cube = scenario.cube;

% Load angular offset
load_in = load('Resources\AngleDopplerOffset.mat');
offset_angle = load_in.offset_smooth;
offset_vel = load_in.vel_axis;

%% Perform Binary Integration

% Find over-threshold detections
bw_cube = (detection.detect_cube_multi > radarsetup.det_m);

% Average power for multiple-detection indices
avg_cube = bw_cube .* (detection.pow_cube_multi ./ detection.detect_cube_multi);
avg_cube(isnan(avg_cube)) = 0;

% Sum over angle information
rd_cube = sum(avg_cube, [3 4]);

%% Determine Individual Object Coordinates

% Find connected objects in R-D cube
cc = bwconncomp(bw_cube);
regions = regionprops(cc, rd_cube, 'WeightedCentroid');

% Generate list of detection coordinates
detection.detect_list.range = [];
detection.detect_list.vel = [];
detection.detect_list.az = [];
detection.detect_list.el = [];
detection.detect_list.cart = [];
detection.detect_list.SNR = [];
detection.detect_list.num_detect = length(regions);

% Reshape power cube to match index format
shape_cube = reshape(avg_cube, [], ...
    length(cube.azimuth_axis), length(cube.elevation_axis));

% Determine Centroid of azimuth-elevation slice
for n = 1:length(regions)
    % Obtain coordinate approximations
    alez = squeeze(sum(shape_cube(cc.PixelIdxList{n}, :, :), 1));
    ang_rgn = regionprops(true(size(alez)), alez, 'WeightedCentroid');
    
    % Store direct coordinates
    detection.detect_list.range(end+1) = interp1(cube.range_axis, regions(n).WeightedCentroid(2));
    detection.detect_list.vel(end+1) = interp1(cube.vel_axis, regions(n).WeightedCentroid(1));
    detection.detect_list.az(end+1) = interp1(cube.azimuth_axis, ang_rgn.WeightedCentroid(1));
    detection.detect_list.el(end+1) = interp1(cube.elevation_axis, ang_rgn.WeightedCentroid(2));
    
    % Correct TDM angle-doppler association
    if strcmp(radarsetup.mimo_type, 'TDM')
        detection.detect_list.az(end) = detection.detect_list.az(end) - ...
            interp1(offset_vel, offset_angle(:,1), detection.detect_list.vel(end), 'linear', 'extrap');
        detection.detect_list.el(end) = detection.detect_list.el(end) - ...
            interp1(offset_vel, offset_angle(:,2), detection.detect_list.vel(end), 'linear', 'extrap');
    end
    
    % Store derived coordinates
    detection.detect_list.cart(:,end+1) = detection.detect_list.range(end) * ...
        [cosd(detection.detect_list.el(end)) * cosd(detection.detect_list.az(end)); ...
        cosd(detection.detect_list.el(end)) * sind(detection.detect_list.az(end));
        sind(detection.detect_list.el(end))];
    
    % Store SNR
    detection.detect_list.SNR(end+1) = 10*log10(max(alez, [], 'all')) ...
        - detection.noise_pow;
end



end

