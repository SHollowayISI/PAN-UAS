function [scenario_out] = RadarSimulation_PANUAS(scenario)
%RADARSIMULATION_PANUAS Generates simulated radar response for PANUAS
%   Takes scenario.sim, .simsetup, .radarsetup, .target_list as inputs and
%   outputs scenario.rx_sig struct containing the received signal.

%% Unpack Variables

scenario_out = scenario;
target_list = scenario.target_list;
sim = scenario.sim;
simsetup = scenario.simsetup;
radarsetup = scenario.radarsetup;
flags = scenario.flags;

%% Setup

% Physical constants
c = physconst('LightSpeed');

% Derived variables
num_tx = radarsetup.n_tx_y * radarsetup.n_tx_z;
num_rx = radarsetup.n_rx_y * radarsetup.n_rx_z;
num_ant = num_tx * num_rx;

%% MIMO Coding Setup

% Generate weighting matrices
switch radarsetup.mimo_type
    case 'TDM'
        weight = eye(num_tx);
    case 'CDM'
        weight = hadamard(num_tx);
    otherwise
        error('ERROR: Specify TDM or CDM for MIMO method');
end

%% Simulation

% Allocate size for signals
rx_sig = zeros(radarsetup.n_s - radarsetup.drop_s, radarsetup.n_p, num_ant);

% Generate the pulse
tx_sig = sim.waveform();

% Transmit the pulse
tx_sig = sim.transmitter(tx_sig);

% Update target to start-of-frame location
% if flags.frame > 1
%     t_cpi = radarsetup.t_ch*radarsetup.n_p*radarsetup.n_tx_y*radarsetup.n_tx_z;
%     [target_list.pos, target_list.vel] = sim.target_plat(t_cpi*(flags.frame-1));
% end


for block = 1:radarsetup.n_p
    
    for chirp = 1:num_tx
        
        % Update target position and velocities
        [target_list.pos, target_list.vel] = sim.target_plat(radarsetup.t_ch);
        
        % Get range and angle to all targets
        [~, tgt_ang] = rangeangle(target_list.pos, simsetup.radar_pos);
        
        % Radiate signal towards all targets
        sig = sim.radiator(tx_sig, tgt_ang, weight(:,chirp));
        
        % Propogate signal to the target through two-way channel
        sig = sim.channel(sig, ...
            simsetup.radar_pos, target_list.pos, ...
            simsetup.radar_vel, target_list.vel);
        
        % Reflect the pulse off of the target
        sig = sim.target(sig, true);
        
        % Collect the reflected target at the antenna array
        sig = sim.collector(sig, tgt_ang);
        
        % Apply receiver noise and gains
        sig = sim.receiver(sig);
        
        % Dechirp signal
        sig = dechirp(sig,tx_sig);
        
        % Apply phase coding to result signal
        mimo_sig = reshape(sig.*permute(weight(:,chirp), [2 3 1]), length(sig), 1, []);
        
        % Save Rx signal by fast time x slow time x virtual antenna
        rx_sig(:,block,:) = rx_sig(:,block,:) + mimo_sig((radarsetup.drop_s+1):end,:,:);
        
    end
    
end


%% Re-pack Variables

scenario_out.rx_sig = rx_sig;

% Note: Do not export target list, since it is modified during


end