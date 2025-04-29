function [y_kp1, A_k, B_k, c_k] = continuous_to_descwete(t_k, t_kp1, y_k, u_k, nx, nu, param)

    % ode shenanigans
    f = @(t, y) eom_discrete_matrix(t, y, u_k, nx, nu, param);
    % disp(t_int);
    % disp(t_k);
    % disp(t_kp1);
    t_int = linspace(t_k, t_kp1);
    opts = odeset("RelTol", 1e-6, "AbsTol", 1e-6);

    traj = ode45(f, t_int, y_k, opts);

    y_kp1 = deval(traj, t_kp1);

    %x_kp1 = y_kp1(1:nx);

    A_k = reshape(y_kp1(nx+1:nx+nx^2), nx, nx);
    B_k = A_k*reshape(y_kp1(nx+nx^2+1:nx+nx^2+nx*nu), nx, nu);
    c_k = A_k*y_kp1(nx+nx^2+nx*nu+1:nx+nx^2+nx*nu+nx);
end

