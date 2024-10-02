function simulate_flight(X0, tspan, m, cd0, k, s, g, rho, cl)
    % This function simulates flight dynamics and plots the results.
    % Parameters:
    % X0 - Initial state vector [x; h; v; gamma]
    % tspan - Time span for simulation [start, end]
    % m - Mass of the aircraft (kg)
    % cd0 - Zero drag coefficient
    % k - Induced drag coefficient
    % s - Reference area (m^2)
    % g - Gravitational acceleration (m/s^2)
    % rho - Air density (kg/m^3)
    % cl - Lift coefficient
    function [value, isterminal, direction] = myEventsFcn(t, X)
        value = X(2);      % We want to stop when y(1) = 0
        isterminal = 1;    % Stop the integration when this event occurs
        direction = 0;     % The zero can be approached from any direction
    end

    options = odeset('Events', @myEventsFcn);

    % Solve the state equations using ode45
    [t, X] = ode45(@(t, X) dy_sim(X, cl, m, cd0, k, s, g, rho), tspan, X0,options);

    % Plot the results
    figure;
    subplot(4, 1, 1);
    plot(t, X(:, 1));
    title('x-coordinate');
    xlabel('Time (s)');
    ylabel('x (m)');

    subplot(4, 1, 2);
    plot(t, X(:, 2));
    title('Altitude');
    xlabel('Time (s)');
    ylabel('h (m)');

    subplot(4, 1, 3);
    plot(t, X(:, 3));
    title('Velocity');
    xlabel('Time (s)');
    ylabel('v (m/s)');

    subplot(4, 1, 4);
    plot(t, X(:, 4));
    title('Flight Path Angle');
    xlabel('Time (s)');
    ylabel('gamma (rad)');

    % Add parameters to the figure title
    params_str = sprintf(['m = %d kg, S = %.2f m^2, \\rho = %.2f kg/m^3, g = %.3f m/s^2\n', ...
        'x_0 = %.2f m, h_0 = %.2f m, v_0 = %.2f m/s, \\gamma_0 = %.2f rad'], ...
        m, s, rho, g, X0(1), X0(2), X0(3), X0(4));
    sgtitle(params_str);
end
