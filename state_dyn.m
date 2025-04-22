% State dynamics function
%{
Inputs:
    x - state
    u - control input
    param - set of all constants and parameters
Outputs:
    dx - state derivative
%}

function dx = state_dyn(x, u, param)
    mu = param.mu;
    I = param.I;
    dx = zeros(7,1);
    dx(1) = x(2);
    dx(2) = mu / abs(x(1))^2 - x(1)*x(4)^2;
    dx(3) = x(4);
    dx(4) = -2*x(2)*x(4)/x(1);
    dx(5) = x(6);
    dx(6) = I \ u;
    dx(7) = dx(6) - dx(4);
end
