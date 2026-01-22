function xk1 = rk4(f,xk,u,h)
%RK4 4th-order Runge Kutta integrator
%   xk1 - state at step k + 1
%   f   - Function handler for dx = f(x,u)
%   xk  - state at step k
%   u   - input at step k
%   h   - integration step
%--------------------------------------------------------------------------

k1 = f(xk,u);
k2 = f(xk+0.5*h*k1,u);
k3 = f(xk+0.5*h*k2,u);
k4 = f(xk+h*k3,u);
xk1 = xk + (1/6).*h.*(k1 + 2*k2 + 2*k3 + k4);
end