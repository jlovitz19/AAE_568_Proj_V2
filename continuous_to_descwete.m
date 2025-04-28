function [y_kp1, A_k, B_k, c_k] = continuous_to_descwete(t_k, t_kp1, y_k, u_k)
    % ode shenanigans
    f = @(t, y) eom_discrete_matrix(t, y, u_k);
    t_int = t_k:t_kp1;
    disp(t_int);
    disp(t_k);
    disp(t_kp1);
    t_int = linspace(t_k, t_kp1);
    opts = odeset("RelTol", 1e-6, "AbsTol", 1e-6);

    traj = ode45(f, t_int, y_k, opts);

    y_kp1 = deval(traj, t_kp1);

    x_kp1 = y_kp1(1:7);

    A_k = reshape(y_kp1(8:56), 7, 7);
    B_k = A_k*reshape(y_kp1(57:63), 7, 1);
    c_k = A_k*y_kp1(64:70);
end

