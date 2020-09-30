%% Housekeeping

clear variables
close all

%% Main Loop

iterations = 10;
vel_list = 0:5:45;

r_out = zeros(length(vel_list), iterations);
v_out = zeros(length(vel_list), iterations);
az_out = zeros(length(vel_list), iterations);
el_out = zeros(length(vel_list), iterations);

for n = 1:length(vel_list)
    
    v_in = vel_list(n);
    r_in = 500;
    el_in = 0;
    az_in = 0;
    rcs_in = 0;
    
    for m = 1:iterations
        
        FullSystem_PANUAS
        
        if scenario.detection.detect_list.num_detect > 0
            
            rs = scenario.detection.detect_list.range;
            vels = scenario.detection.detect_list.vel;
            [~, I] = min((v_in - vels).^2 + (r_in - rs).^2);
            
            r_out(n,m) = scenario.detection.detect_list.range(I);
            v_out(n,m) = scenario.detection.detect_list.vel(I);
            az_out(n,m) = scenario.detection.detect_list.az(I);
            el_out(n,m) = scenario.detection.detect_list.el(I);
        else
            r_out(n,m) = nan;
            v_out(n,m) = nan;
            az_out(n,m) = nan;
            el_out(n,m) = nan;
        end
        
    end
    
%     r_avg = mean(r_out, 2, 'omitnan');
%     v_avg = mean(v_out, 2, 'omitnan');
%     az_avg = mean(az_out, 2, 'omitnan');
%     el_avg = mean(el_out, 2, 'omitnan');
    
    
end


%% Plotting

% close all
% 
% figure;
% plot(velocities, az_avg, velocities, el_avg)
% grid on
% legend('Azimuth Error', 'Elevation Error')
% xlabel('Range Rate [m/s]', 'FontWeight', 'bold')
% ylabel('Angle Error [deg]', 'FontWeight', 'bold')

