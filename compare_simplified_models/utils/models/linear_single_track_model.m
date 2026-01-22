function [x_dot,log] = linear_single_track_model(x,u,car)

%% vectors

% state
vy = x(1);
r = x(2);

% input
vx = u(1);
Delta = u(2);

%% vehicle paramteres
m = car.m; % [Kg] -> vehicle mass
Iz = car.Iz; % [Kg*m^2] -> vehicle inertia
lf = car.lf; % [m] distance of CoG from front axle
lr = car.lr; % [m] distance of CoG from rear axle
Fzf0 = m*car.g*lr/(lf+lr);
Fzr0 = m*car.g*lf/(lf+lr);
Fzf_aero = 0.5*car.rho*vx.^2*car.Cz*car.S*car.aero_split_f;
Fzr_aero = 0.5*car.rho*vx.^2*car.Cz*car.S*(1-car.aero_split_f);
Fzf = Fzf0 + Fzf_aero;
Fzr = Fzr0 + Fzr_aero;

car.Kf = (car.tire_f.d * car.tire_f.c * car.tire_f.b);
car.Kr = (car.tire_r.d * car.tire_r.c * car.tire_r.b);

Cf = car.Kf*Fzf; % front cornering stiffness
Cr = car.Kr*Fzr; % rear cornering stiffness

%% vehicle model in MATRIX FORM
% vx = max(1,vx);
% A11 = -(Cf+Cr)/(m*vx);
% A12 = -(Cf*lf-Cr*lr)/(m*vx) - vx; 
% A21 = -(Cf*lf-Cr*lr)/(Iz*vx); 
% A22 = -(Cf*lf^2+Cr*lr^2)/(Iz*vx);
% 
% B1 = Cf/m;
% B2 = Cf*lf/Iz;

% dx - matrix model
% vy_dot = + A11*vy + A12*r + B1*Delta;
% r_dot = + A21*vy + A22*r + B2*Delta;

%% vehicle model in EQUATION FORM
alpha_f = ((vy + r*car.lf)/vx) - Delta;
alpha_r = ((vy - r*car.lr)/vx);

% Linear tire model
Fyf = (alpha_f) * Cf;
Fyr = (alpha_r) * Cr;

% dx
vy_dot = 1/car.m * (Fyr + Fyf*cos(Delta) - car.m*vx*r);
r_dot = 1/car.Iz * (- Fyr*car.lr + Fyf*cos(Delta)*car.lf);

x_dot = [vy_dot; r_dot];

%% store variable
log = [vy, r, alpha_f, alpha_r, Fyf, Fyr, Fzf, Fzr];