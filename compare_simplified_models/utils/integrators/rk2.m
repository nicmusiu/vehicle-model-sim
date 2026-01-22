function xk1 = rk2(f, xk, u, h)
%RK2 2nd-order Runge Kutta integrator
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

% Combine stages
xk1 = xk + h*k2;
end
