clc; clear; close all;

% define physical parameters
G = 6.6743e-11; % Gravitational constant (m/s^)/(kg/m^2)
param.Me = 5.9722e24;  % Earth mass (kg)
param.Re = 6.37836e6; % Radius of the Earth (can be 1 for normalized sphere)
param.m = 61.6;
param.mu = G*(param.Me+param.m);
param.I = 1;
param.max_thrust = .03;
param.max_torque = .5;

% orbit parameters
param.M = 0;
param.a = 460e3+param.Re; % semi major axis (m)
param.e = 0;

% gains
param.w1 = 1;
param.w2 = 1;
param.w3 = 1;
param.w4 = 1;

% get 1 period
tf = 2*pi*sqrt(param.a^3 / (G*param.Me));

% get translational ICs in polar for translation
trans_IC = kep2polar([param.a;param.e;param.M], param);

% extract the ones we care about
r0 = trans_IC(1); dr0 = trans_IC(3); theta0 = trans_IC(2); dtheta0 = trans_IC(4);
param.alt = r0;

% define attitude ICs
phi0 = deg2rad(10); % this is read as "pointing 10 degrees up from x axis"
dphi0 = deg2rad(0); % starting with 0 angular velocity
psi0 = phi0 - theta0;

e_2_0 = 0;

% form IC vector
x0 = [r0; dr0; theta0; dtheta0; phi0; dphi0; psi0; e_2_0];
param.x0 = [x0; zeros(8,1)];

% solve bvp
solinit = bvpinit(linspace(0,tf,1000), param.x0);
options = bvpset('Stats', 'on', 'RelTol', 1e-3);
sol = bvp4c(@(t, y) bvp_ode(t, y, param), @(ya, yb) bvp_bcs(ya, yb, param), solinit, options);

% Recover state history and calculate control history
x = sol.y;
t = sol.x;
u1 = -x(10, :)/(2*param.w3);
u2 = -x(14,:)/(2*param.w4*param.I);

% Plot control history

figure
sgtitle("Control Time History", "Interpreter", "Latex", "FontSize", 20)
subplot(2, 1, 1);
plot(t(1:45),u1(1:45));
grid on
title("$u_{1}(t)$", "Interpreter", "Latex")
xlabel("time [s]", "Interpreter", "Latex", "FontSize", 15)
ylabel("u [N-m]", "Interpreter", "Latex", "FontSize", 15)
subplot(2, 1, 2);
plot(t(1:45),u2(1:45));
grid on;
title("$u_{2}(t)$", "Interpreter", "Latex")
xlabel("time [s]", "Interpreter", "Latex", "FontSize", 15)
ylabel("u [N-m]", "Interpreter", "Latex", "FontSize", 15)

% Plot state error history
figure
plot(t(1:45),x(7,1:45)); grid on;
title("Pointing Error vs. Time", "Interpreter", "Latex", "FontSize", 20)
xlabel("time [s]", "Interpreter", "Latex", "FontSize", 15)
ylabel("$\psi$ [rad]", "Interpreter", "Latex", "FontSize", 15)

% Recover polar coordinates
rho = x(1,:);
theta = x(3,:);

x_cart = rho.*cos(theta);
y_cart = rho.*sin(theta);

figure
plot(x_cart/1000,y_cart/1000,"r"); hold on;
plot(param.Re*cos(theta)/1000,param.Re*sin(theta)/1000,"b")
title("2D Circular Orbit Plot", "Interpreter", "Latex", "FontSize", 20)
xlabel("x [km]", "Interpreter", "Latex", "FontSize", 15)
ylabel("y [km]", "Interpreter", "Latex", "FontSize", 15)
legend(["Orbit", "Earth Radius"])
grid on;
axis equal