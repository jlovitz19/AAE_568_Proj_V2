% Runge kutta 4 method for discretization
%{
Inputs:
    f - state dynamics function
    x - state
    u - control input
    dt - time step --> specified by user
Outputs:
    x_kp1 - estimate of state at next time step
%}

function x_kp1 = rk4(f, x, u, dt)
    k1 = f(x, u);
    k2 = f(x + 0.5*dt*k1, u);
    k3 = f(x + 0.5*dt*k2, u);
    k4 = f(x + dt*k3, u);
    x_kp1 = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
end
