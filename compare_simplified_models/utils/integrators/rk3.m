function xk1 = rk3(f, xk, u, h)
%RK3 3rd-order Runge Kutta integrator
%   xk1 - state at step k + 1
%   f   - Function handler for dx = f(x,u)
%   xk  - state at step k
%   u   - input at step k
%   h   - integration step
%--------------------------------------------------------------------------

% First stage
k1 = f(xk, u);
% Second stage
k2 = f(xk + 0.5*h*k1, u);
% Third stage
k3 = f(xk - h*k1 + 2*h*k2, u);
% Combine stages
xk1 = xk + (h/6)*(k1 + 4*k2 + k3);
end
