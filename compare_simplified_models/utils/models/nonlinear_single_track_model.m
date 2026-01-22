function [dx, log] = nonlinear_single_track_model(x,u,car)              % generic solver (rk4,euler)

%% Vectors
% state
vy = x(1);
r = x(2);
% input
vx = u(1);
Delta = u(2);
% additional variables
ax = u(3);

%% Aerodynamic force
downforce_f = 0.5*car.rho*vx.^2*car.Cz*car.S*car.aero_split_f;
downforce_r = 0.5*car.rho*vx.^2*car.Cz*car.S*(1-car.aero_split_f);

%% Vertical load
Fzf0 = car.m*car.g*car.cg;
Fzr0 = car.m*car.g*(1-car.cg);
dFz_long = car.m * car.h_G * ax / car.wb;

Fzf = Fzf0 + downforce_f - dFz_long;
Fzr = Fzr0 + downforce_r + dFz_long;

%% Congruence Equations
% vx_min = max(vx, 0.5); 
vx_min = vx;
alpha_f = atan((vy + r*car.lf)/vx_min) - Delta;
alpha_r = atan((vy - r*car.lr)/vx_min);

%% Lateral tires force

% Fyf = Fzf * (car.tire_f.sv + car.tire_f.d*sin(car.tire_f.c*atan((car.tire_f.b*(alpha_f + car.tire_f.sh)))) - ...
%             car.tire_f.e*((car.tire_f.b*(alpha_f + car.tire_f.sh))-atan(car.tire_f.b*(alpha_f + car.tire_f.sh))));
% Fyr = Fzr * (car.tire_r.sv + car.tire_r.d*sin(car.tire_r.c*atan((car.tire_r.b*(alpha_r + car.tire_r.sh)))) - ...
%             car.tire_r.e*((car.tire_r.b*(alpha_r + car.tire_r.sh))-atan(car.tire_r.b*(alpha_r + car.tire_r.sh))));

Fyf = Fzf * (car.tire_f.sv + car.tire_f.d*sin(car.tire_f.c*atan((car.tire_f.b*(alpha_f)))));
Fyr = Fzr * (car.tire_r.sv + car.tire_r.d*sin(car.tire_r.c*atan((car.tire_r.b*(alpha_r)))));
ay=(Fyf+Fyr)/car.m;

%% dx
vy_dot = 1/car.m * (Fyr + Fyf*cos(Delta) - car.m*vx*r);
r_dot = 1/car.Iz * (Fyf*cos(Delta)*car.lf - Fyr*car.lr);

dx = [vy_dot; r_dot];

%% store variable
log = [ ...
    vx, ... % 1
    vy, ... % 2
    r, ... % 3
    Delta, ... % 4
    alpha_f, ... % 5
    alpha_r, ... % 6
    Fyf, ... % 7
    Fyr, ... % 8
    Fzf, ... % 9
    Fzr, ... % 10
    ay ... % 11
    ];