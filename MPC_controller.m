function MPC_controller(len, Q, R, p, n_s, x0, A, B, c)
% Parameters
% len: length of time vector
% Q: state weight matrix
% R: input weight matrix
% p: parameters
% n_s: number of states and inputs
% x0: initial conditions
% A: linearized A matrix
% B: linearized B matrix
% c: linearized c matrix

    i = "interpreter";
    ii = "latex";
    
    % get important vector lengths
    n_x = n_s(1);
    n_u = n_s(2);

    % horizon and associated interval
    N = 10;
    t_interval = 1:1/len:N;

    % create MPC weight matrices
    Q_bar = kron(eye(N), Q);
    R_bar = kron(eye(N), R);

    % MPC constraint weights
    E = -[1 0; -1 0; 0 1; 0 -1; 1 0; -1 0; 0 1; 0 -1];
    W = -[p.max_thrust; p.max_thrust;...
        -p.max_thrust; -p.max_thrust;...
        p.max_torque; p.max_torque;...
        -p.max_torque; -p.max_torque];
    
    % random stuff
    iA = false(size(W));

    opt = mpcActiveSetOptions;
    opt.IntegrityChecks = false;

    % initialize states
    u_MPC = NaN(n_u, length(t_interval));
    x_MPC = NaN(n_x, length(t_interval));
    x_MPC(:, 1) = x0;
   

    for n = 1:len
        x = x_MPC(:, n);

        H_k = create_H_MPC(A(:, :, n), N, n_x);
        G_k = create_G_MPC(A(:, :, n), B(:, :, n), N, n_x);

        F_k = G_k'*Q_bar*H_k;
        L_k = G_k'*Q_bar*G_k + R_bar;

        [u_mpc, ~, iA] = mpcActiveSetSolver(L_k, F_k*x, E, W,...
            [], zeros(0, 1), iA, opt);

        u_MPC(:, n+1) = u_mpc(:, 1);

        u_MPC(:, n+1) = A(:, :, n)*x_MPC(:, n) + B(:, :, n)*u_mpc(:, 1);
        y_MPC = c(:, n)*x_MPC(:, n);
    end

    k = 1:1:len;

    figure;
    plot(x_MPC(1, :).*cos(x_MPC(3, :)), x_MPC(1, :).*sin(x_MPC(3, :)));
    grid on;
    title("Trajectory of Satellite", i, ii);
    xlabel("$x_{1}$", i, ii);
    ylabel("$x_{2}$", i, ii);

    figure
    sgtitle("Control Input Time History", i, ii);

    subplot(2, 1, 1);
    plot(k, u_MPC(1, :));
    grid on;
    xlabel("$k$", i, ii);
    ylabel("$u_{1}$", i, ii);

    subplot(2, 1, 2);
    plot(k, u_MPC(2, :));
    grid on;
    xlabel("$k$", i, ii);
    ylabel("$u_{2}$", i, ii);
end