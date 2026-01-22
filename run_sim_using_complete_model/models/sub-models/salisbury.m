% -------------------------------------------------------------------------
% salisbury.m - compute limited slip differential output (longitudinal rear axle forces)
%
% Copyright (c) 2025 [Unimore Racing]
% All rights reserved.
% 
% Author: [nicola.musiu@unimore.it]
% Date: 2025-03-12
% -------------------------------------------------------------------------

% parameters
preload = car.preload;
locking = car.locking / 100;

% estinguish Differential momentum
vx_rl = vx + r * (+car.tr / 2);
vx_rr = vx + r * (-car.tr / 2);
dOmega = (vx_rl - vx_rr);

ki = 10.0;
xi = 1 - exp(-ki * abs(dOmega));

% lateral load transfer
M_roll = car.m * ay * car.q;
dFzr_lat = Fyr * car.h_RRC / car.tr + car.Kr_eq * M_roll / car.tr;
Fzrl = Fzr/2 - dFzr_lat; 
Fzrr = Fzr/2 + dFzr_lat;

Feng = Feng;

M_diff_pos = max(preload, locking * Feng) * sign(Delta); % Trazione
M_diff_neg = - min(-preload, locking * Feng) * sign(Delta); % Rilascio

if Feng >= 0
    M_diff = M_diff_pos;
else
    M_diff = M_diff_neg;
end

M_diff = M_diff * xi;

% compute limit
Fxrl_max = Fzrl * car.Dx_r;
Fxrr_max = Fzrr * car.Dx_r;

% Force at wheels
Fxrl = Feng/2 + M_diff/2;
Fxrr = Feng/2 - M_diff/2;

Fxrl_esuberant = Fxrl - Fxrl_max;
Fxrr_esuberant = Fxrr - Fxrr_max;

Fxrl = min(Fxrl, Fxrl_max);
Fxrr = min(Fxrr, Fxrr_max);

if Fxrl == Fxrl_max
    Fxrr = min(Fxrr + Fxrl_esuberant, Fxrr_max);
elseif Fxrr == Fxrr_max
    Fxrl = min(Fxrl + Fxrr_esuberant, Fxrl_max);
end

Fxr = Fxrr + Fxrl;
