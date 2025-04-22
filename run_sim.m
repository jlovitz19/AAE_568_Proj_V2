clc; clear; close all;

% define physical parameters
param.G = 6.6743e-11; % Gravitational constant (m/s^)/(kg/m^2)
param.Me = 5.9722e24;  % Earth mass (kg)
param.Re = 6.37836e6; % Radius of the Earth (can be 1 for normalized sphere)
param.m = 61.6;
param.mu = param.G*(param.Me+param.m);

% orbit parameters
param.M = 0;
param.a = 460e3+param.Re; % semi major axis (m)
param.e = 0;

% get translational ICs in polar for translation
trans_IC = kep2polar([param.a;param.e;param.M], param);

% extract the ones we care about
r0 = trans_IC(1); dr0 = trans_IC(3); theta0 = trans_IC(2); dtheta0 = trans_IC(4);

% define attitude ICs
phi0 = deg2rad(10); % this is read as "pointing 10 degrees up from x axis"
dphi0 = deg2rad(0); % starting with 0 angular velocity
psi0 = phi0 - theta0;

% form IC vector
x0 = [r0; dr0; theta0; dtheta0; phi0; dphi0; psi0];








