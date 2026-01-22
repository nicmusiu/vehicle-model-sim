function xk1 = euler(f,xk,u,h)
% explicit euler integrator
%   xk1 - state at step k + 1
%   f   - Function handler for dx = f(x,u)
%   xk  - state at step k
%   u   - input at step k
%   h   - integration step
%--------------------------------------------------------------------------
xk1 = xk + h*f(xk,u);
end