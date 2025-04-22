function plot_u_time_history(t, u, fig_num)
    % function that plots the time history of the controller
    %
    % Parameters:
    % t: time interval 
    % u: control input
    % fig_num: number of figure
    %
    % Outputs: None
    
    i = "interpreter";
    ii = "latex";

    figure(fig_num);

    plot(t, u, "b");

    title("Control Time History");
    xlabel("$t$ (s)", i, ii);
    ylabel("$u(t)$ (N*m)", i, ii);

    grid on;
end