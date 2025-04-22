function bcs = bvp_bcs(yi, yf, p)
    % boundary conditions for two-point boundary value problem
    %
    % Paramters:
    % yi: initial states and costates
    % yf: final states and costates
    % p: parameters
    %
    % Outputs:
    % bcs: boundary conditions

    % initial conditions
    %%% NEEDS TO BE UPDATED WITH ACTUAL IC's %%%
    x_t0 = [yi(1) yi(2) yi(3) yi(4) yi(5) yi(6) yi(7)];

    % fixed final states
    %%% NEEDS TO BE UPDATED WITH ACTUAL FC's %%%
    x_tf_fixed = [yf(1) yf(2) yf(3) yf(4)];

    % E-L equations
    lambda_tf = [yf(12) yf(13) yf(14)];

    bcs = [x_t0 x_tf_fixed lambda_tf];
end