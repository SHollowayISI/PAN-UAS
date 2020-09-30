%% PANUAS Radar System - End of Simulation Tasks
%{

    Sean Holloway
    PANUAS End-of-Simulation Tasks

    Container script which saves output files from PANUAS simulation,and
    sends alert email.

    For use in FullSystem_PANUAS without automated simulation.
    
%}

%% Announce Elapsed Time

toc

%% Save Files and Figures

% Establish file name
save_name = [scenario.simsetup.filename, '_', datestr(now, 'mmddyy_HHMM')];

% Establish filepaths for saving
mat_path = 'MAT Files\Scenario Objects\';
fig_path = ['Figures\', save_name, '\'];

% Save scenario object if chosen
if scenario.simsetup.save_mat
    SaveScenario(scenario, save_name, mat_path);
end

% Save open figures if chosen
if scenario.simsetup.save_figs
    for ftype = 1:length(scenario.simsetup.save_format.list)
        SaveFigures(save_name, fig_path, scenario.simsetup.save_format.list{ftype});
    end
end


%% Send Email Alert

% Send email alert with attachment if chosen
if scenario.simsetup.send_alert
    
    % Set up email process
    EmailSetup();
    
    % Send email
    EmailAlert( ...
        scenario.simsetup.alert_address, ...
        save_name, ...
        scenario.simsetup.attach_zip);
end


%% File Management (AUTOMATED TEST ONLY)

%{
% Move file to "Complete" folder
movefile(['Automated Testing/To Run/', scenario_filename], ...
    'Automated Testing/Complete/');

% Display message to command window
disp(['Simulation scenario complete: ', scenario_filename]);
%}










