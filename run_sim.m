clc; clear; close all;

% define orbit
R_e = 6.378e6;
a = R_e + 4600; % r
ecc = 0; % eccentricity
M = 0; % mean anomaly

% get translational ICs in polar for translation
trans_IC = kep2polar([a;ecc;M]);

% extract the ones we care about
r0 = trans_IC(1); dr0 = trans_IC(3); theta0 = trans_IC(2); dtheta0 = trans_IC(4);

% define attitude ICs
phi0 = deg2rad(10); % this is read as "pointing 10 degrees up from x axis"
dphi0 = deg2rad(0); % starting with 0 angular velocity
psi0 = phi0 - theta0;

% form IC vector
x0 = [r0; dr0; theta0; dtheta0; phi0; dphi0; psi0];








