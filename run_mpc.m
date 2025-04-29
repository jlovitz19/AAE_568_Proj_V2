%%
% MPC stuff
u_bar = [u1; u2];

% discwetize
y_k = [x0; reshape(eye(8), 64, 1); zeros(24, 1)];
A = NaN(8, 8, length(t));
B = NaN(8, 2, length(t));
c = NaN(8, length(t));

% Dimensions
nx = size(A,1); % Dimension of state
nu = size(B,2); % Dimension of input

x_k = NaN(8, length(t));
k_d = 1:1:length(t);
x_k(:, 1) = x0;

for k = 1:length(t)-1
    u_k = u_bar(:, k);

    [y_kp1, A_k, B_k, C_k] = continuous_to_descwete(t(k), t(k+1), y_k, u_k, nx, nu);

    x_k(:, k) = y_kp1(1:nx);
    A(:, :, k) = A_k;
    B(:, :, k) = B_k;
    c(:, k) = C_k;

    y_k = y_kp1;
end

figure
plot(x_k(1, :).*cos(x_k(3, :)), x_k(1, :).*sin(x_k(3, :)));
grid on;



load('A.mat');
load('B.mat');
load('c.mat');
%%

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
