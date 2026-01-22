clear all, close all; clc;

addpath utils/ params/ ../compare_simplified_models/utils/models/
car = yaml.loadFile("model.yaml").params;
settings = jsondecode(fileread("settings.json"));

%% PERFORM STABILITY ANALYSIS

syms vx vy r Delta ax
x = [vy; r];
u = [vx; Delta; ax];

% Generate the symbolic dynamic function
if settings.use_dynamic_linear
    [x_dot,~] = linear_single_track_model(x, u, car);
elseif settings.use_dynamic_non_linear
    [x_dot,~] = nonlinear_single_track_model(x, u, car);
end

% Jacobians for linearization
A = jacobian(x_dot, x);  % Derivative of f with respect to the state vector x
B = jacobian(x_dot, u);  % Derivative of f with respect to the input vector u

% Test results
evaluate_jacobians_at_specific_operating_point

%% USE compare_simplified_models RESULTS!
load ../compare_simplified_models/results.mat;

vx_values = results.nonlinear_single_track.vx;  % Speed in m/s
Delta_values = results.nonlinear_single_track.Delta;  % Steering angle range
vy_values = results.nonlinear_single_track.vy;
r_values = results.nonlinear_single_track.r;
is_stable = true; % Initialize a flag for instability
all_poles = [];
all_polesd = [];

% Loop through each combination of operating points
for i=1:height(vx_values)
    % Set the operating point
    x0 = [vy_values(i); r_values(i)];  % Assuming steady-state lateral velocity and yaw rate
    u0 = [vx_values(i); Delta_values(i); 0.0];

    % Compute linearized matrices at this operating point
    A_lin = double(subs(A, [x; u], [x0; u0]));
    B_lin = double(subs(B, [x; u], [x0; u0]));

    % Create state-space model
    C = eye(length(x)); % Full state observation
    D = zeros(length(x), length(u)); % Assuming no direct feedthrough
    sys = ss(A_lin, B_lin, C, D);
    sysd = c2d(sys,settings.h);

    % Get and store poles
    poles = pole(sys);
    polesd = pole(sysd);

    % Check for any poles with positive real parts (unstable)
    if any(real(poles) > 0)
        disp(['System may be unstable at vx = ', num2str(vx_values(i)), ' m/s and Delta = ', num2str(Delta_values(i)), ' rad']);
        is_stable = false;
    end

    all_poles = [all_poles; poles];
    all_polesd = [all_polesd; polesd];
end

if is_stable
    disp('System appears stable across the tested range of conditions.');
end

% integrator stability
integrators_stability_region(settings.h)
plot(real(all_poles), imag(all_poles), 'x', 'LineWidth', 2, 'MarkerSize', 10);
% discrete time stability
discrete_time_stability_region(all_polesd)