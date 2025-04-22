% Discretization Test
% Example runner script for runge kutta 4 discretization
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

param.I = 1;

dt = 1;

x0 = [param.a; 0; 0; sqrt(param.mu / param.a^3)*param.a; 0; 0; 0];  % initial state (7x1)
u0 = 0;  % assuming 2D control input torque

x_next = rk4(@(x,u) state_dyn(x,u,param), x0, u0, dt);