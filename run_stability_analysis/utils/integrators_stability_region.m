function f = integrators_stability_region(timestep)
% load system matrix from simulated model
h = timestep;
% h = 0.01; % set a different timestep to check the effective stability region

% Define stability functions for different integrators
euler_stab = @(z) abs(1 + h*z) - 1;
rk2_stab = @(z) abs(1 + h*z + 1/2*(h*z).^2) - 1;
rk3_stab = @(z) abs(1 + h*z + 1/2*(h*z).^2 + 1/6*(h*z).^3) - 1;
rk4_stab = @(z) abs(1 + h*z + 1/2*(h*z).^2 + 1/6*(h*z).^3 + 1/24*(h*z).^4) - 1;

% Evaluate the stability functions over the range of values
l = 400; % 100(@ts=50ms), 1000(@ts=10ms), ...
n = l;
x = linspace(-l, l, n);
y = linspace(-l, l, n);
[X,Y] = meshgrid(x, y);
Z = X + 1j*Y;

euler_f = euler_stab(Z);
Rk2_f = rk2_stab(Z);
Rk3_f = rk3_stab(Z);
Rk4_f = rk4_stab(Z);

% % constant vx
figure
hold on, grid on, grid minor;
contour(X, Y, euler_f, [0 0], 'LineWidth', 2 , 'color', 'r');
contour(X, Y, Rk2_f, [0 0], 'LineWidth', 2 , 'color', 'b');
contour(X, Y, Rk3_f, [0 0], 'LineWidth', 2 , 'color', 'k');
contour(X, Y, Rk4_f, [0 0], 'LineWidth', 2 , 'color', 'g');
axis([-l l -l l]);
xlabel('Real Part');
ylabel('Imaginary Part');
title('Stepsize - h =' + string(h) + 's');
legend('Euler','Runge-Kutta 2','Runge-Kutta 3','Runge-Kutta 4','Location','northwest');
