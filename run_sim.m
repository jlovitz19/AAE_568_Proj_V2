clc; clear; close all;

% define physical parameters
param.G = 6.6743e-11; % Gravitational constant (m/s^)/(kg/m^2)
param.Me = 5.9722e24;  % Earth mass (kg)
param.Re = 6.37836e6; % Radius of the Earth (can be 1 for normalized sphere)
param.m = 61.6;
param.mu = param.G*(param.Me+param.m);
param.I = 1;
param.max_torque = .5;

% orbit parameters
param.M = 0;
param.a = 460e3+param.Re; % semi major axis (m)
param.e = 0;

% gains
param.w1 = 1;
param.w2 = 1;

% get 1 period
tf = 2*pi*sqrt(param.a^3 / (param.G*param.Me));

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
param.x0 = [x0;zeros(7,1)];

% solve bvp
solinit = bvpinit(linspace(0,tf,1000), param.x0);
options = bvpset('Stats', 'on', 'RelTol',1e-3);
sol = bvp4c(@(t, y) bvp_ode(t, y, param), @(ya, yb) bvp_bcs(ya, yb, param), solinit, options);

% Recover state history and calculate control history
x = sol.y;
t = sol.x;
u = -x(13,:)/(2*param.w1*param.I);

% Plot control history
plot_u_time_history(t,u,1)