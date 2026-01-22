% -------------------------------------------------------------------------
% sim_model.m - state space model implementation of TRICYCLE model
%
% Copyright (c) 2025 [Unimore Racing]
% All rights reserved.
% 
% Author: [nicola.musiu@unimore.it]
% Date: 2025-03-12
% -------------------------------------------------------------------------

function [dx,log] = sim_model(x, u, car, engine, settings)

%% STATE VECTOR
s = x(1);
n = x(2);
mu = x(3);
vx = x(4);
vy = x(5);
r = x(6);

%% INPUT VECTOR
Delta = u(1);
D = u(2);
B = u(3);
gear = u(4);

%% INDEPENDENT VARIABLES
kappa = 0; % TODO: forward kappa from trajectory (or compute it step by step)
ax = 0; % TODO: compute acc_x as derivative of Vx!
ay = vx * r;

%% SLIP ANGLES
scaling_factor = exp(-max(0, car.vx_kin_dyn_blend - vx)); % Exponential decay to estinguish dynamic quantities at low speed.
alpha_f = scaling_factor * (atan((vy + r*car.lf)/max(car.vx_zero,vx)) - Delta);
alpha_r = scaling_factor * atan((vy - r*car.lr)/max(car.vx_zero,vx));

%% VERTICAL LOAD
Fzf0 = car.m*car.g*car.lr/car.L;
Fzr0 = car.m*car.g*car.lf/car.L;

Fzf_aero = 0.5*car.rho*vx.^2*car.Cz*car.S*car.aero_split_front;
Fzr_aero = 0.5*car.rho*vx.^2*car.Cz*car.S*(1-car.aero_split_front);

dFz_long = car.m*ax*car.h/car.L; 

Fzf = Fzf0 + Fzf_aero - dFz_long;
Fzr = Fzr0 + Fzr_aero + dFz_long;

Fzf = max(0,Fzf);
Fzr = max(0,Fzr);

%% LATERAL FORCES
Fyf = Fzf * (car.Svy_f + car.Dy_f*sin(car.Cy_f*atan((car.By_f*(alpha_f + car.Shy_f))))); - ...
             car.Ey_f*((car.By_f*(alpha_f + car.Shy_f))-atan(car.By_f*(alpha_f + car.Shy_f)));
Fyr = Fzr * (car.Svy_r + car.Dy_r*sin(car.Cy_r*atan((car.By_r*(alpha_r + car.Shy_r))))); - ...
             car.Ey_r*((car.By_r*(alpha_r + car.Shy_r))-atan(car.By_r*(alpha_r + car.Shy_r)));
Fyf = Fyf * scaling_factor;
Fyr = Fyr * scaling_factor;

Fyf_lat = Fyf;
Fyr_lat = Fyr;

%% ENGINE TORQUE
tau = (car.gear_ratio)*car.final_ratio;
rpm = min(car.rpm_limiter,max(900,vx*tau(gear)/car.wh_radius*car.RADS_2_RPM));

rpm_ratio = rpm / car.rpm_limiter;
torque_scale = max(0, 1 - exp(500 * (rpm_ratio - 1))); % Attenuazione della coppia a max RPM
Torque = getEngineTorque(engine,rpm,D) * torque_scale;

%% LONGITUDINAL FORCES
Fd = - 0.5*car.rho*vx.^2*car.Cx*car.S * (vx > 0);
Feng = Torque.*tau(gear)/car.wh_radius;

Fxf = - car.CBf*B * (vx > 0); % TODO: include longitudinal slip ratio!
Fxr = Feng - (car.CBr*B) * (vx > 0); % TODO: include longitudinal slip ratio!

%% OPEN DIFFERENTIAL
Fxrl = Fxr/2; 
Fxrr = Fxr/2;

%% SALISBURY DIFFERENTIAL
if(settings.LSD)
    salisbury
end
M_yaw = (Fxrr - Fxrl) * car.tr / 2;

%% COMBINED SLIP
Fxf_limit = abs(Fxf/(car.Dx_f*Fzf)); % if > 1, tires are sliding. if = 1, tires reach the grip limit.
force_ratio_f = min(Fxf_limit,0.99);
Gyf = cos(asin(force_ratio_f)); % if = 0.99, tires are on their max performance. if -> 0, combined slip decrease the lateral grip.

Fxr_limit = abs(Fxr/(car.Dx_r*Fzr));
force_ratio_r = min(Fxr_limit,0.99);
Gyr = cos(asin(force_ratio_r));

if(settings.combined_slip)
    Fyf = Fyf*Gyf;
    Fyr = Fyr*Gyr;
end

% enforce tire longitudinal limit!
Fxf = max(Fxf, - car.Dx_f*Fzf); % Braking force
Fxr = min(Fxr, + car.Dx_r*Fzr); % Traction force
Fxr = max(Fxr, - car.Dx_r*Fzr); % Braking force

%% EOM
s_dot = (vx*cos(mu) - vy*sin(mu))/(1 - n*kappa);
n_dot = vy * cos(mu) + vx * sin(mu);
vx_dot = 1/car.m * (Fd + Fxr - Fyf*sin(Delta) + Fxf*cos(Delta) + car.m*vy*r);

% use DYNAMIC model
mu_dot = r - kappa * (vx * cos(mu) - vy * sin(mu)) / (1 - n * kappa);
vy_dot = 1/car.m * (Fyr + Fxf*sin(Delta) + Fyf*cos(Delta) - car.m*vx*r);
r_dot = 1/car.Iz * (M_yaw - Fyr*car.lr + (Fxf*sin(Delta) + Fyf*cos(Delta))*car.lf);

% use KINEMATIC model
dDelta = 0; % TODO: include steering derivate calculation for kinematic model
mu_dot_kin = vx/car.L * tan(Delta) - kappa * (vx * cos(mu)) / (1 - n * kappa);
vy_dot_kin = (dDelta*vx + Delta*vx_dot)*car.lr/car.L;
r_dot_kin = (dDelta*vx + Delta*vx_dot)/car.L;

% Model blending
lambda_raw = 1.0 - exp(-(vx - car.vx_kin_dyn_blend) * car.kin_dyn_blend_smoothing);
lambda = max(0.0, lambda_raw);

mu_dot = lambda*mu_dot + (1-lambda)*mu_dot_kin;
vy_dot = lambda*vy_dot + (1-lambda)*vy_dot_kin;
r_dot = lambda*r_dot + (1-lambda)*r_dot_kin;

dx = [s_dot; n_dot; mu_dot; vx_dot; vy_dot; r_dot];

%% STORE VARIABLES

ax = vx_dot;

log = [s, ... % 1
       n, ... % 2
       mu, ... % 3
       vx, ... % 4
       vy, ... % 5
       r, ... % 6
       ax, ... % 7
       Delta, ... % 8
       D, ... % 9
       B, ... % 10
       alpha_f,... % 11
       alpha_r,... % 12
       Fyf,... % 13
       Fyr,... % 14
       Fzf,... % 15
       Fzr,... % 16
       Fyf_lat,... % 17
       Fyr_lat,... % 18
       Gyf,... % 19
       Gyr,... % 20
       Fxf,... % 21
       Fxrl,... % 22 
       Fxrr,... % 23 
       M_yaw,...% 24
       rpm,... % 25
       Fzrl,... % 26 
       Fzrr,... % 27
       Torque,...% 28
       Feng % 29
       ];