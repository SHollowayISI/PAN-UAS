function [SNR] = CalculateSNR(scenario,RCS,Range)
%CALCULATESNR Calculates SNR of target for PANUAS scenario
%   Takes radar scenario object, RCS (absolute) value, and Range 
%   as input, provides SNR value as output.

%% Unpack Variables
radarsetup = scenario.radarsetup;

%% Calculate SNR

n_tx = radarsetup.n_tx_y*radarsetup.n_tx_z;
n_rx = radarsetup.n_rx_y*radarsetup.n_rx_z;

lambda = physconst('LightSpeed')/radarsetup.f_c;
t_cpi = radarsetup.t_ch*radarsetup.n_p*n_tx;
Lr = db2pow(radarsetup.rf_sys_loss);
NF = db2pow(radarsetup.rx_nf);
c = (4*pi)^3;
n = physconst('Boltzmann')*290;

Gt = radarsetup.ant_gain;
% Gr = radarsetup.ant_gain*n_tx*n_rx;
Gr = Gt;

switch radarsetup.mimo_type
    case 'TDM'
        total_pow = radarsetup.tx_pow;
    case 'CDM'
        total_pow = radarsetup.tx_pow * n_tx;
end


SNR_abs = (total_pow * Gt * Gr * t_cpi * lambda * lambda * RCS) ...
    ./ (c * (Range.^4) * n * NF * Lr);

SNR = pow2db(SNR_abs);

end

