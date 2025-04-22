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
    gamma_min = 5;
    gamma_max = 20;

    % retrieving states
    x1 = y(1);
    x2 = y(2);
    x3 = y(3);
    x4 = y(4);
    x5 = y(5);
    x6 = y(6);
    x7 = y(7);

    % retrieving costates
    lambda1 = y(8);
    lambda2 = y(9);
    lambda3 = y(10);
    lambda4 = y(11);
    lambda5 = y(12);
    lambda6 = y(13);
    lambda7 = y(14);

    % retrieve parameters
    I = p.I;
    a = p.a;
    b = p.b;
    mu = p.mu;

    % apply input constraints
    u = .5*lambda6/(I*b);

    if u - gamma_max > 0
        u = u - gamma_max;
    elseif gamma_min - u > 0
        u = gamma_min - u;
    end

    % find x_dot
    x_dot1 = x2;
    x_dot2 = mu/abs(x1^2) - x1*x4^2;
    x_dot3 = x4;
    x_dot4 = -2*x2*x4/x1;
    x_dot5 = x6;
    x_dot6 = u/I;
    x_dot7 = x6 - x4;

    x_dot = [x_dot1; x_dot2; x_dot3; x_dot4; x_dot5; x_dot6; x_dot7];

    % find lambda_dot
    lambda_dot1 = -x2*x4^2 - 2*lambda2*mu/(x1^3) + lambda4*2*x2*x4/(x1^2);
    lambda_dot2 = lambda1 - 2*lambda4*x4/x1;
    lambda_dot3 = 0;
    lambda_dot4 = -2*lambda2*x1*x4 - 2*lambda2*x2/x2 - lambda7;
    lambda_dot5 = 0;
    lambda_dot6 = lambda5 + lambda7;
    lambda_dot7 = 2*a*x7;

    lambda_dot = -[lambda_dot1; lambda_dot2; lambda_dot3; lambda_dot4;...
        lambda_dot5; lambda_dot6; lambda_dot7];

    % combine state and costate derivatives
    y_dot = [x_dot; lambda_dot];
end