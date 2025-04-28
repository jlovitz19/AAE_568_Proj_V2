function bcs = bvp_bcs(yi, yf, param)
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
    x_t0 = [yi(1) - param.x0(1);
        yi(2) - param.x0(2);
        yi(3) - param.x0(3);
        yi(4) - param.x0(4);
        yi(5) - param.x0(5);
        yi(6) - param.x0(6);
        yi(7) - param.x0(7);
        yi(8) - param.x0(8)];

    % fixed final states
    %%% NEEDS TO BE UPDATED WITH ACTUAL FC's %%%
%     x_tf_fixed = [
%         yf(1) - yi(1);
%         yf(2) - yi(2);
%         yf(3) - yi(3);
%         yf(4) - yi(4)
%         ];

    % E-L equations
    lambda_tf = [yf(9);
        yf(10);
        yf(11);
        yf(12);
        yf(13);
        yf(14);
        yf(15);
        yf(16)];

    bcs = [x_t0; lambda_tf];

end