clc; clear; close all;

% define physical parameters
param.G = 6.6743e-11; % Gravitational constant (m/s^)/(kg/m^2)
param.Me = 5.9722e24;  % Earth mass (kg)
param.Re = 6.37836e6; % Radius of the Earth (can be 1 for normalized sphere)
param.m = 61.6;
param.mu = param.G*(param.Me+param.m);
param.I = 1;
param.max_beta = 15;
param.max_torque = .5;

% orbit parameters
param.M = 0;
param.a = 460e3+param.Re; % semi major axis (m)
param.e = 0;

% gains
param.w1 = 1;
param.w2 = 1;
param.w3 = 1;

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
param.x0 = [x0; zeros(7,1)];

% solve bvp
solinit = bvpinit(linspace(0,tf,1000), param.x0);
options = bvpset('Stats', 'on', 'RelTol',1e-3);
sol = bvp4c(@(t, y) bvp_ode(t, y, param), @(ya, yb) bvp_bcs(ya, yb, param), solinit, options);

% Recover state history and calculate control history
x = sol.y;
t = sol.x;
u = -x(13,:)/(2*param.w1*param.I);

% Plot control history

figure
plot(t(1:45),u(1:45))
grid on;
title("Control Time History", "Interpreter", "Latex", "FontSize", 20)
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


% MPC stuff
u_bar = u;

% discwetize
y_k = [x0; reshape(eye(7), 49, 1); zeros(14, 1)];
A = NaN(7, 7, length(t));
B = NaN(7, length(t));
c = NaN(7, length(t));

x_k = NaN(7, length(t));
k_d = 1:1:length(t);
x_k(:, 1) = x0;

for k = 1:length(t)-1
    u_k = u_bar(k);

    [y_kp1, A_k, B_k, C_k] = continuous_to_descwete(t(k), t(k+1), y_k, u_bar(k));

    x_k(:, k) = y_kp1(1:7);
    A(:, :, k) = A_k;
    B(:, k) = B_k;
    c(:, k) = C_k;

    y_k = y_kp1;
end

figure
plot(x_k(1, :).*cos(x_k(3, :)), x_k(1, :).*sin(x_k(3, :)));
grid on;



load('A.mat');
load('B.mat');
load('c.mat');

%{

% implement MPC
N = 30;
tv = 1:1/length(t):N;

Q_bar = kron(eye(N), Q);
R_bar = kron(eye(N), R);

E = -[1 0; -1 0; 0 1; 0 -1];
W = -[param.max_torque; param.max_torque;...
    -param.max_torque; -param.max_torque];

iA = false(size(W));

opt = mpcActiveSetOptions;
opt.IntegrityChecks = false;

u_MPC = NaN(1, length(tv));
x_MPC = NaN(7, length(tv));
x_MPC(:, 1) = x0;

for j = 1:length(tv)-1
    x = x_MPC(:, j);

    if isnan(A(1,1,j))
        disp(penis)
    end
    H = create_H_MPC(A(:, :, j), N);
    G = create_G_MPC(A(:, :, j), B(:, j), N);

    F = G'*Q_bar*H;
    L = G'*Q_bar*G + R_bar;

    [u_mpc, status, iA] = mpcActiveSetSolver(L, F*x, E, W,...
        [], zeros(0, 1), iA, opt);

    u_MPC(j+1) = u_mpc(1);

    x_MPC(:, j+1) = A(:, :, j)*x_MPC(:, j) + B(:, j)*u_MPC(j + 1);
    y_MPC = c(:, j)'*x_MPC(:, j);
end

figure(3)
plot(x_MPC(1, :).*cos(x_MPC(3, :)), x_MPC(1, :).*sin(x_MPC(3, :)));
grid on;

figure(69)
plot(k_d, u_MPC);
grid on;
%}
