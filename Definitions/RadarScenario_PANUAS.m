% ClassDef File for PANUAS Radar Scenario

classdef RadarScenario_PANUAS < handle
    properties
        target_list
        simsetup
        radarsetup
        sim
        rx_sig
        cube
        detection
        flags
        timing
        results
        multi
    end
    
    methods
        
        function RadarScenario = RadarScenario_PANUAS()
            % Initialize structure of target list
            RadarScenario.target_list = struct( ...
                'pos',      [], ...
                'vel',      [], ...
                'rcs',      []);
            
            RadarScenario.multi.detect_list = {};
            
            RadarScenario.multi.track_list = {};
            
            RadarScenario.multi.active_tracks = [];
            
            RadarScenario.detection.detect_cube_multi = [];
        end
       
        function generateCoordinates(RadarScenario)
            %Generate Input Coordinate Grid
            [range_grid, azimuth_grid, elevation_grid] = meshgrid( ...
                RadarScenario.cube.range_axis, ...
                RadarScenario.cube.azimuth_axis, ...
                RadarScenario.cube.elevation_axis);
            %Generate Output Coordinate Grid
            RadarScenario.results.x_grid = ...
                permute( ...
                range_grid .* cosd(azimuth_grid) .* cosd(elevation_grid), ...
                [2 1 3]);
            RadarScenario.results.y_grid = ...
                permute( ...
                range_grid .* sind(azimuth_grid) .* cosd(elevation_grid), ...
                [2 1 3]);
            RadarScenario.results.z_grid = ...
                permute( ...
                range_grid .* sind(elevation_grid), ...
                [2 1 3]);
        end
        
        function timeStart(RadarScenario)
            % Begin timing for progress readout
            RadarScenario.timing.timing_logical = true;
            RadarScenario.timing.startTime = tic;
            RadarScenario.timing.TimeDate = now;
            RadarScenario.timing.numLoops = ...
                RadarScenario.simsetup.num_frames * ...
                RadarScenario.radarsetup.cpi_fr;
            RadarScenario.timing.timeGate = 0;
        end
        
        function timeUpdate(RadarScenario, repetition, rep_method)
            
            if ~RadarScenario.timing.timing_logical
                error('Must use method timeStart() before timeUpdate()');
            end
            
            % Calculate progress through simulation
            loops_complete = (RadarScenario.flags.frame-1)*RadarScenario.radarsetup.cpi_fr + ...
                RadarScenario.flags.cpi;
            percent_complete = 100*loops_complete/RadarScenario.timing.numLoops;
            
            % Calculate remaining time in simulation
            nowTimeDate = now;
            elapsedTime = nowTimeDate - RadarScenario.timing.TimeDate;
            estComplete = nowTimeDate + ((100/percent_complete)-1)*elapsedTime;
            
            % Form message to display in command window
            message_l = sprintf('%d Loops complete out of %d', loops_complete, RadarScenario.timing.numLoops);
            message_p = [sprintf('Percent complete: %0.0f', percent_complete), '%'];
            message_t = ['Estimated time of completion: ', datestr(estComplete)];
            
            % Display current progress
            switch rep_method
                case 'loops'
                    if (mod(loops_complete, repetition) == 1) || (repetition == 1)
                        disp(message_l);
                        disp(message_p);
                        disp(message_t);
                        disp('');
                    end
                    
                case 'time'
                    if ((RadarScenario.timing.timeGate == 0) || ...
                            (toc > repetition + RadarScenario.timing.timeGate))
                        disp(message_p);
                        disp(message_t);
                        disp('');
                        RadarScenario.timing.timeGate = toc;
                    end
                    
            end
        end
        
        function CPIUpdate(RadarScenario)
            message = sprintf('CPI %d complete out of %d per frame.', ...
                RadarScenario.flags.cpi, ...
                RadarScenario.radarsetup.cpi_fr);
            disp(message);
        end
        
        function readOut(RadarScenario)
%             fprintf('\nMaximum SNR: %0.1f [dB]\n', ...
%                 RadarScenario.detection.max_SNR);
            
            num_detect = RadarScenario.detection.detect_list.num_detect;
            
            if num_detect > 0
                if num_detect > 1
                    fprintf('\n%d Targets Detected:\n\n', num_detect);
                else
                    fprintf('\n%d Target Detected:\n\n', num_detect);
                end
                
                for n = 1:num_detect
                    fprintf('Target #%d Coordinates:\n', n);
                    fprintf('Range: %0.1f [m]\n', ...
                        RadarScenario.detection.detect_list.range(n));
                    fprintf('Velocity: %0.1f [m/s]\n', ...
                        RadarScenario.detection.detect_list.vel(n));
                    fprintf('Azimuth Angle: %0.1f [deg]\n', ...
                        RadarScenario.detection.detect_list.az(n));
                    fprintf('Elevation Angle: %0.1f [deg]\n\n', ...
                        RadarScenario.detection.detect_list.el(n));
                end
            else
                disp('No Targets Detected');
                disp('');
            end
        end
        
        function saveMulti(RadarScenario)
            
            RadarScenario.multi.detect_list{RadarScenario.flags.frame} = ...
                RadarScenario.detection.detect_list;
            
        end
        
        function viewTargets(RadarScenario)
            % Show 3-D scatter plot of target locations
            figure('Name', 'Target 3D Scatter Plot')
            scatter3(RadarScenario.target_list.pos(1,:), ...
                RadarScenario.target_list.pos(2,:), ...
                RadarScenario.target_list.pos(3,:))
        end
        
        function viewRDCube(RadarScenario, graphType)
            if strcmp(graphType, 'heatmap')
                figure('Name', 'Range-Doppler Heat Map');
                imagesc(RadarScenario.cube.vel_axis, ...
                    RadarScenario.cube.range_axis, ...
                    10*log10(RadarScenario.cube.pow_cube(:,:,ceil(end/2), ceil(end/2))))
                title('Range-Doppler Heat Map')
                set(gca,'YDir','normal')
                xlabel('Velocity [m/s]','FontWeight','bold')
                ylabel('Range [m]','FontWeight','bold')
            else
                figure('Name', 'Range-Doppler Surface');
                surf(RadarScenario.cube.vel_axis, ...
                    RadarScenario.cube.range_axis, ...
                    10*log10(RadarScenario.cube.pow_cube(:,:,ceil(end/2), ceil(end/2))), ...
                    'EdgeColor', 'none')
                title('Range-Doppler Surface')
                set(gca,'YDir','normal')
                xlabel('Velocity [m/s]','FontWeight','bold')
                ylabel('Range [m]','FontWeight','bold')
                zlabel('FFT Log Intensity [dB]','FontWeight','bold')
            end
            
        end
        
        function viewRACube(RadarScenario, graphType)
            switch graphType
                case 'heatmap'
                    figure('Name', 'Range-Azimuth Heat Map');
                    imagesc(RadarScenario.cube.azimuth_axis, ...
                        RadarScenario.cube.range_axis, ...
                        10*log10(squeeze(RadarScenario.cube.pow_cube(:,ceil(end/2),:, ceil(end/2)))))
                    title('Range-Azimuth Heat Map')
                    set(gca,'YDir','normal')
                    xlabel('Azimuth Angle [degree]','FontWeight','bold')
                    ylabel('Range [m]','FontWeight','bold')
                case 'surface'
                    figure('Name', 'Range-Azimuth Surface');
                    surf(RadarScenario.cube.azimuth_axis, ...
                        RadarScenario.cube.range_axis, ...
                        10*log10(squeeze(RadarScenario.cube.pow_cube(:,ceil(end/2),:, ceil(end/2)))), ...
                        'EdgeColor', 'none')
                    title('Range-Azimuth Surface')
                    xlabel('Azimuth Angle [degree]','FontWeight','bold')
                    ylabel('Range [m]','FontWeight','bold')
                    zlabel('FFT Log Intensity [dB]','FontWeight','bold')
                case 'PPI'
                    figure('Name', 'Range-Azimuth PPI');
                    surf(RadarScenario.cube.x_grid, ...
                        RadarScenario.cube.y_grid, ...
                        10*log10(squeeze(RadarScenario.cube.pow_cube(:,ceil(end/2),:, ceil(end/2)))), ...
                        'EdgeColor', 'none')
                    title('Range-Azimuth PPI')
                    xlabel('Cross-Range Distance [m]','FontWeight','bold')
                    ylabel('Down-Range Distance [m]','FontWeight','bold')
                    zlabel('FFT Log Intensity [dB]','FontWeight','bold')
            end
            
        end
        
        function viewDetections(RadarScenario, graphType)
            switch graphType
                case 'heatmap'
                    figure('Name', 'Detection Heatmap')
                    imagesc(RadarScenario.cube.vel_axis, ...
                        RadarScenario.cube.range_axis, ...
                        RadarScenario.detection.detect_cube( ...
                        :, :, ceil(end/2), ceil(end/2)))
                    set(gca, 'YDir', 'Normal')
                    xlabel('Velocity [m/s]', 'FontWeight', 'bold')
                    ylabel('Ragne [m]', 'FontWeight', 'bold')
                case 'PPI'
                    figure('Name', 'Detection PPI');
                    surf(RadarScenario.cube.x_grid, ...
                        RadarScenario.cube.y_grid, ...
                        RadarScenario.detection.detect_cube_nodop( ...
                        :, :, RadarScenario.flags.slice), ...
                        'EdgeColor', 'none')
                    view(270,90)
                    title('Range-Azimuth PPI')
                    xlabel('Cross-Range Distance [m]','FontWeight','bold')
                    ylabel('Down-Range Distance [m]','FontWeight','bold')
                    zlabel('FFT Log Intensity [dB]','FontWeight','bold')
            end
        end
        
        function viewDetections3D(RadarScenario)
            % Pass in variable
            detect_list = RadarScenario.multi.detect_list;
            % Show 3-D scatter plot of detections
            figure('Name', 'Detections 3D Scatter Plot')
            for n = 1:length(detect_list)
                scatter3(detect_list{n}.cart(1,:), ...
                    detect_list{n}.cart(2,:), ...
                    detect_list{n}.cart(3,:), ...
                    'k', '+');
                hold on;
            end
            title('Detections 3D Scatter Plot')
            xlabel('Down-Range Distance [m]', 'FontWeight', 'bold')
            ylabel('Cross-Range Distance [m]', 'FontWeight', 'bold')
            zlabel('Elevation [m]', 'FontWeight', 'bold')
            
        end
        
        function viewTracking(RadarScenario)
            % Pass in variables
            track_list  = RadarScenario.multi.track_list;
            % Generate plot
            figure('Name', 'Tracking Results Scatter Plot');
            % Add tracks to plot
            for n = 1:length(track_list)
                % Scatter plot if false alarm
                if track_list{n}.false_alarm
                    scatter3(track_list{n}.det_list(1,:), ...
                        track_list{n}.det_list(2,:), ...
                        track_list{n}.det_list(3,:), ...
                        'r', '+');
                    hold on;
                % Line of track if not
                else
                    scatter3(track_list{n}.det_list(1,:), ...
                        track_list{n}.det_list(2,:), ...
                        track_list{n}.det_list(3,:), ...
                        'k', '+');
                    hold on;
                    plot3(track_list{n}.est_list(1,:), ...
                        track_list{n}.est_list(3,:), ...
                        track_list{n}.est_list(5,:), ...
                        'g');
                    hold on;
                end
            end
            % Add radar location to plot
            scatter3(0, 0, 0, 'filled', 'r');
            % Correct plot limits
            ax = gca;
            ax.YLim = [-ax.XLim(2)/2, ax.XLim(2)/2];
            ax.ZLim = [-ax.XLim(2)/2, ax.XLim(2)/2];
            % Add labels
            xlabel('Down Range Distance [m]', 'FontWeight', 'bold')
            ylabel('Cross Range Distance [m]', 'FontWeight', 'bold')
            zlabel('Altitude [m]', 'FontWeight', 'bold')
        end
             
    end
end






