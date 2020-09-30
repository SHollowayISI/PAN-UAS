
close all
clear pts

z = err.*movmean(err,3)' + randn(length(err))*0.005;
z = z + 0.1*(1 - (1:length(err))'/length(err)) + 0.15*(1 - (1:length(err))/length(err));
z = conv2(z, eye(5), 'valid'); 
z = 2*z - 3;

zlen = size(z,1);

figure;
z_ax = linspace(0, 10, zlen);
v_ax = linspace(0, 2.5, zlen);
surf(v_ax, z_ax, z, 'EdgeColor', 'none')
hold on;

lift = 0.1;

start = [60; 70];
pts = [start; z(start(2), start(1)) + lift];

curr = start;


for step = 1:3
    
    
    for coord = 1:2
        
        len = ceil(zlen/(2^(step-1)));
        
        ln_start = max(1, floor(curr(coord) - len/2));
        ln_stop = min(zlen, ceil(curr(coord) + len/2));
        
        line = curr .* ones(2, ln_stop - ln_start + 1);
        line(coord,:) = ln_start:ln_stop;
        
        if coord == 2
            line(3,:) = z(line(2,:), curr(1)) + lift;
        else
            line(3,:) = z(curr(2), line(1,:)) + lift;
        end
        
        line(3,:) = movmean(line(3,:), 3);
        
        plot3(v_ax(line(1,:)), z_ax(line(2,:)), line(3,:), 'r')
        
        [~, I] = min(line(3,:));
        
        curr = line(1:2,I);
        pts(:,end+1) = [curr; line(3,I)];
        
    end
    
end

txt_lift = 0.1;

init_cost = pts(3,1) - lift;
msg = sprintf('     Initial Value: Cost = %0.3f', init_cost);
% text(pts(1,1), pts(2,1), pts(3,1)+txt_lift, msg)
ann_in = annotation('textbox', 'String', msg, 'FitBoxToText', 'on', 'BackgroundColor', 'w');

end_cost = pts(3,end) - lift;
msg = sprintf('     Final Value: Cost = %0.3f', end_cost);
% text(pts(1,end), pts(2,end), pts(3,end)+txt_lift, msg)
ann_end = annotation('textbox', 'String', msg, 'FitBoxToText', 'on', 'BackgroundColor', 'w');

scatter3(v_ax(pts(1,:)), z_ax(pts(2,:)), pts(3,:), 'filled', 'y')
xlabel('Measurement Variance, \sigma_z', 'FontWeight', 'bold')
ylabel('Kinematic Process Variance, \sigma_v', 'FontWeight', 'bold')
zlabel('Cost Function Value', 'FontWeight', 'bold')
title('Coordinate Descent Optimization')


