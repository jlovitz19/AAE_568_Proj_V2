%{
Convert Keplerian elements to cartesian position and velocity vectors
Inputs:
  a             - semi-major axis [m]
  e             - eccentricity
  i             - inclination [rad]
  omeg          - right ascension of ascending node [rad]
  wumbo         - argument of periapsis [rad]
  M             - mean anomaly [rad]
Outputs:
  x             - State vector in ECI frame including position and
  velocity
%}

function x_polar = kep2polar(x,m)

% Process inputs
if nargin < 2
   m = 61.6; % If no input given for m, assume zero
end

x_polar = NaN(2,length(x));

for idx = 1:length(x)
x_kep = x(:,idx);

% Retrieve orbital elements
a=x_kep(1); e=x_kep(2); M=x_kep(3);

% Constants
G = 6.6743e-11; % Gravitational constant (m/s^)/(kg/m^2)
Me = 5.9722e24;  % Earth mass (kg)
mu = G*(Me+m);   % Gravitational constant mu

% Solve for true anomaly
E = solve_kepler(M,e); % Eccentric anomaly
cos_nu = (cos(E) - e) / (1 - e*cos(E));
sin_nu = (sqrt(1 - e^2) * sin(E)) / (1 - e*cos(E));
nu = atan2(sin_nu, cos_nu);

% Compute rho
r = a * (1 - e^2) ./ (1 + e * cos(nu));

% 1st derivatives
dE_dM = 1 ./ (1 - e * cos(E));
dnu_dE = sqrt(1 - e^2) ./ (1 - e * cos(E));
dnu = dnu_dE .* dE_dM * n;

drho_dnu = a * (1 - e^2) .* e .* sin(nu) ./ (1 + e * cos(nu)).^2;
dr = drho_dnu .* dnu;

% 2nd derivatives
ddnu = -2*dr*dnu/r;
ddr = mu/r^2 - r*dnu^2;

% In our coordinate system, nu = theta
theta = nu;
dtheta = dnu;
ddtheta = ddnu;

x_polar(:,idx) = [r;theta;dr;dtheta;ddr;ddtheta];

end

end