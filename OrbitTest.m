clc; close; clear;
param.G = 6.6743e-11; % Gravitational constant (m/s^)/(kg/m^2)
param.Me = 5.9722e24;  % Earth mass (kg)
param.Re = 6.37836e6; % Radius of the Earth (can be 1 for normalized sphere)
param.a = 460e3+param.Re; % semi major axis (m)
param.m = 61.6;
param.mu = param.G*(param.Me+param.m);

[t,x_kep] = sim_orbit([param.a;0.00054;deg2rad(304.8477)],param);
x_polar = kep2polar(x_kep,param);

rho = x_polar(1,:);
theta = x_polar(2,:);
drho = x_polar(3,:);
dtheta = x_polar(4,:);
ddrho = x_polar(5,:);
ddtheta = x_polar(6,:);

figure
subplot(2,1,1)
plot(t,rho); grid on;
subplot(2,1,2)
plot(t,theta); grid on;