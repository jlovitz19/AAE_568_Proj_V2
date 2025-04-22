clc; close; clear;
param.G = 6.6743e-11; % Gravitational constant (m/s^)/(kg/m^2)
param.Me = 5.9722e24;  % Earth mass (kg)
param.Re = 6.37836e6; % Radius of the Earth (can be 1 for normalized sphere)
param.a = 460e3+param.Re; % semi major axis (m)
param.m = 61.6;
param.mu = param.G*(param.Me+param.m);

[t,x_kep] = sim_orbit([param.a;0;0],param);
x_polar = kep2polar(x_kep,param);

rho = x_polar(1,:);
theta = x_polar(2,:);
drho = x_polar(3,:);
dtheta = x_polar(4,:);
ddrho = x_polar(5,:);
ddtheta = x_polar(6,:);

% Plot polar components

figure
sgtitle("Polar components vs. time", "Interpreter", "Latex")
names= ["$\rho$ [m]","$\theta$ [rad]","$\dot{\rho}$ [m/s]",...
    "$\dot{\theta}$ [rad/s]","$\ddot{\rho}$ $[\frac{rad}{s^2}]$",...
    "$\ddot{\theta}$ $[\frac{rad}{s^2}]$"];
for idx = 1:size(x_polar,1)
subplot(3,2,idx)
plot(t,x_polar(idx,:)); grid on;
title(names(idx) + ' vs. time', "Interpreter", "Latex")
end

% Plot entire orbit

figure
polarplot(theta,rho);
title("Orbit visualization", "Interpreter", "Latex")
grid on;