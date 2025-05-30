function y_dot = bvp_ode(t, y, p)
    % is the controller function that solves the two-point boundary value
    % problem using calculus of variations
    %
    % Paramters:
    % t: time index for given ode call
    % y: vector containing state and costate
    % p: parameters
    %
    % Outputs:
    % y_dot: derivative of vector constaining state and costate

    % input limits
    beta_min = -p.max_thrust;
    beta_max = p.max_thrust;

    gamma_min = -p.max_torque;
    gamma_max = p.max_torque;

    % retrieving states
    x1 = y(1);
    x2 = y(2);
    x3 = y(3);
    x4 = y(4);
    x5 = y(5);
    x6 = y(6);
    x7 = y(7);
    x8 = y(8);

    % retrieving costates
    lambda1 = y(9);
    lambda2 = y(10);
    lambda3 = y(11);
    lambda4 = y(12);
    lambda5 = y(13);
    lambda6 = y(14);
    lambda7 = y(15);
    lambda8 = y(16);

    % retrieve parameters
    I = p.I;
    a1 = p.w1;
    b1 = p.w2;
    a2 = p.w3;
    b2 = p.w4;
    mu = p.mu;
    %alt = param.alt;

    % apply input constraints
    u1 = -.5*lambda2/b1;

    if u1  > beta_max
        u1 = beta_max;
    elseif u1 < beta_min
        u1 = beta_min;
    end

    u2 = -.5*lambda6/(I*b2);

    if u2  > gamma_max
        u2 = gamma_max;
    elseif u2 < gamma_min
        u2 = gamma_min;
    end

    % find x_dot
    x_dot1 = x2;
    x_dot2 = -mu/x1^2 + x1*x4^2 + u1;
    x_dot3 = x4;
    x_dot4 = -2*x2*x4/x1;
    x_dot5 = x6;
    x_dot6 = u2/I;
    x_dot7 = x6 - x4;
    x_dot8 = x2;

    x_dot = [x_dot1; x_dot2; x_dot3; x_dot4; x_dot5; x_dot6; x_dot7; x_dot8];

    % find lambda_dot
    lambda_dot1 = lambda2*(2*mu/x1^3 + x4^2) +...
        lambda4*2*x2*x4/x1^2 + lambda8;
    lambda_dot2 = -2*lambda4*x4/x1 + lambda1;
    lambda_dot3 = 0;
    lambda_dot4 = 2*lambda2*x1*x4 + lambda3 - lambda4*2*x2/x1 - lambda7;
    lambda_dot5 = 0;
    lambda_dot6 = lambda5 + lambda7;
    lambda_dot7 = 2*a1*x7;
    lambda_dot8 = 2*a2*x8;

    %{
    lambda_dot1 = -x2*x4^2 - 2*lambda2*mu/(x1^3) + lambda4*2*x2*x4/(x1^2);
    lambda_dot2 = lambda1 - 2*lambda4*x4/x1;
    lambda_dot3 = 0;
    lambda_dot4 = -2*lambda2*x1*x4 - 2*lambda2 - lambda7;
    lambda_dot5 = 0;
    lambda_dot6 = lambda5 + lambda7;
    lambda_dot7 = 2*w1*x7;
    %}

    lambda_dot = -[lambda_dot1; lambda_dot2; lambda_dot3; lambda_dot4;...
        lambda_dot5; lambda_dot6; lambda_dot7; lambda_dot8];

    % combine state and costate derivatives
    y_dot = [x_dot; lambda_dot];
end