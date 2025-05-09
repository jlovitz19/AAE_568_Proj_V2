%{
Two body dynamics of 2d satellite orbit

Inputs
State "x" representing classical orbital elements:
    a: semi-major axis (m)
    e: eccentricty
    M: mean anomaly (rad)
param: contains all parameters and constants

Outputs
x_dot: the time derivative of the state

Assumptions:
 - Earth centered orbit
 - Using radians

Note: must add "solve_kepler.m" to path in main runner script
%}

function x_dot = Orbit_EOM_2D(x,param)

% Retrieve orbital elements
a=x(1);

% Process inputs
if nargin < 2
    m = 61.6; % If no input given for m, assume 61.6
    G = 6.6743e-11; % Gravitational constant (km/s^)/(kg/m^2)
    Me = 5.9722e24;  % Earth mass (kg)
else
    m = param.m;
    G = param.G;
    Me = param.Me;
end

% Mean motion
n = sqrt(G*(Me+m)/a^3);
f0 = [0,0,n]';

x_dot = f0;

end