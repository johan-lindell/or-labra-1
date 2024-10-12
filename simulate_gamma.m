% Script to simulate and plot flight dynamics for different initial flight path angles

% Clear workspace and figures
clear all;
close all;
clc;

% First set of parameters
X0 = [0; 240; 5; 0];    % Initial state [x, h, v, gamma], gamma will be varied
tspan = [0, 100];       % Time span [start, end]
m = 100;                % Mass in kg
cd0 = 0.034;            % Zero drag coefficient
k = 0.07;               % Induced drag coefficient
s = 14;                 % Reference area in m^2
g = 9.81;               % Gravitational acceleration in m/s^2
rho = 1.13;             % Air density in kg/m^3
cl = 0;                 % Lift coefficient

% Array of initial flight path angles to test (in radians)
gamma0_array = deg2rad([-50, -20, 0, 20, 50]);  % Convert degrees to radians

% Initialize figure and subplots
figureHandle = figure;

% Flight Path subplot
subplot(3, 1, 1);
hold on;
title('Flight Path');
xlabel('x (m)');
ylabel('h (m)');

% Velocity subplot
subplot(3, 1, 2);
hold on;
title('Velocity');
xlabel('Time (s)');
ylabel('v (m/s)');

% Flight Path Angle subplot
subplot(3,1,3);
hold on;
title('Flight Path Angle');
xlabel('Time (s)');
ylabel('\gamma (rad)');

% Prepare ODE options with event function
options = odeset('Events', @myEventsFcn);

% Loop over each initial flight path angle
for idx = 1:length(gamma0_array)
    % Set the initial flight path angle
    gamma0 = gamma0_array(idx);
    X0(4) = gamma0;  % Update initial gamma

    % Solve the state equations using ode45
    [t, X] = ode45(@(t, X) dy_sim(X, cl, m, cd0, k, s, g, rho), tspan, X0, options);

    % Create label for this simulation
    label_str = sprintf('\\gamma_0 = %.1f°', rad2deg(gamma0));  % Convert rad to degrees for labeling

    % Plot Flight Path
    subplot(3, 1, 1);
    plot(X(:,1), X(:,2), 'DisplayName', label_str);

    % Plot Velocity
    subplot(3, 1, 2);
    plot(t, X(:, 3), 'DisplayName', label_str);

    % Plot Flight Path Angle
    subplot(3,1,3);
    plot(t, X(:, 4), 'DisplayName', label_str);
end

% Add legends to the plots
subplot(3, 1, 1);
legend show;
subplot(3, 1, 2);
legend show;
subplot(3,1,3);
legend show;

% Add parameters to the figure title
params_str = sprintf(['Simulation for different initial flight path angles\n' ...
    'm = %d kg, S = %.2f m^2, \\rho = %.2f kg/m^3, g = %.3f m/s^2\n', ...
    'x_0 = %.2f m, h_0 = %.2f m, v_0 = %.2f m/s'], ...
    m, s, rho, g, X0(1), X0(2), X0(3));
sgtitle(params_str);

% Function for the event (stopping condition)
function [value, isterminal, direction] = myEventsFcn(t, X)
    value = X(2);       % Stop when altitude h = 0
    isterminal = 1;     % Stop the integration
    direction = -1;     % Negative direction only
end