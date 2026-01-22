function [x_dot,log] = kinematic_model(x,u,car, settings)

%% vectors
% state
X = x(1);
Y = x(2);
psi = x(3);
% input
v = u(1);
Delta = u(2);

%% vehicle paramteres
lf = car.lf;
lr = car.lr;
car.wb = lf + lr;

if (settings.is_kinematic_model_enhanced)
    Fz_f_0 = car.m*car.g*car.cg;
    Fz_r_0 = car.m*car.g*(1-car.cg);
    Fz_f_aero = 0.5*car.rho*v.^2*car.Cz*car.S*car.aero_split_f;
    Fz_r_aero = 0.5*car.rho*v.^2*car.Cz*car.S*(1-car.aero_split_f);
    Fzf = Fz_f_0 + Fz_f_aero;
    Fzr = Fz_r_0 + Fz_r_aero;
    Cf = (-car.tire_f.b*car.tire_f.c*car.tire_f.d)*Fzf; % front cornering stiffness
    Cr = (-car.tire_r.b*car.tire_r.c*car.tire_r.d)*Fzr; % rear cornering stiffness
    Kus = (car.m / car.wb / car.wb) * ((Cr*car.lr - Cf*car.lf) / (Cf*Cr));
else
    Kus = 0;
end

%% motion equations
beta = atan((lr*tan(Delta))/car.wb); % [rad] slip angle calculated at each cycle

X_dot = v*cos(psi);
Y_dot = v*sin(psi);
psi_dot = (v)/car.wb * tan(Delta) / (1 + Kus * v * v);

x_dot = [X_dot;
          Y_dot;
          psi_dot];

%% store variables
log = [X, Y, psi, beta, X_dot, Y_dot, psi_dot];
