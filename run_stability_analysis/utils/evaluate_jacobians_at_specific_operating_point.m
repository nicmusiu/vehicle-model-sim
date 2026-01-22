
% Evaluate the Jacobians at a specific operating point 
x0 = [0.0; 0.0];  % Example steady state
u0 = [10; 0.0; 0.0];  % Set vx0 (e.g., 10 m/s) and Delta0 = 0

A_lin = double(subs(A, [x; u], [x0; u0]));
B_lin = double(subs(B, [x; u], [x0; u0]));
C = eye(length(x)); % Full state observation
D = zeros(length(x), length(u)); % Assuming no direct feedthrough

% generate the dynamic system
sys = ss(A_lin, B_lin, C, D);
sysd = c2d(sys,settings.h);

% show results
if isstable(sys)
    stability = true;
    disp('System appears stable across the tested range of conditions.');
else
    disp('System shows potential instability at certain conditions.');
end

poles = pole(sys);
polesd = pole(sysd);

integrators_stability_region(settings.h) % create a PLOT with the stability region for main solvers
plot(real(poles), imag(poles), 'x', 'LineWidth', 2, 'MarkerSize', 12);
discrete_time_stability_region(polesd)