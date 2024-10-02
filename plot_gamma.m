% Define constants
CD0 = 0.034;
K = 0.07;

% Define the function gamma as a function of CL
gamma = @(CL) -(CD0 + K * CL.^2) ./ CL;

% Define the objective function we want to maximize: 1 / -gamma
objective_function = @(CL) 1 ./ -gamma(CL);

% Generate a range for CL values
CL_values = linspace(-1.5, 1.5, 1000);

% Calculate the objective function for these CL values
objective_values = objective_function(CL_values);

% Plot the objective function
figure;
plot(CL_values, objective_values, 'LineWidth', 2);
xlabel('C_L');
ylabel('1/ -\gamma');
title('1 / -\gamma');
grid on;

% Highlight critical points (CL = ±0.696932 and ±1.4)
hold on;
plot([-0.696932, 0.696932], objective_function([-0.696932, 0.696932]), 'ro', 'MarkerSize', 10);
plot([-1.4, 1.4], objective_function([-1.4, 1.4]), 'go', 'MarkerSize', 10);
legend('1 / -\gamma', 'C_L = ±0.696932', 'C_L = ±1.4');
hold off;


% Define constants
CD0 = 0.034;
K = 0.07;

% Define the drag coefficient as a function of CL
CD = @(CL) CD0 + K * CL.^2;

% Generate a range of CL values
CL_values = linspace(-1, 1, 1000);

% Calculate corresponding CD values
CD_values = CD(CL_values);

% Plot the (CL, CD) curve
figure;
plot(CL_values, CD_values, 'LineWidth', 2);
hold on;

% Find the point where the tangent passes through the origin
CL_tangent = sqrt(CD0 / K);  % This is derived from d(CD)/d(CL) = CD/CL

% Calculate the slope of the tangent (which is CD / CL at that point)
slope_tangent = CD(CL_tangent) / CL_tangent;

% Plot the tangent passing through the origin
plot(CL_values, slope_tangent * CL_values, '--r', 'LineWidth', 2);

% Plot the specific point on the (CL, CD) curve
plot(CL_tangent, CD(CL_tangent), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

% Labels and titles
xlabel('C_L');
ylabel('C_D');
title('(C_L, C_D) Curve');
grid on;

% Add a legend
legend('(C_L, C_D) Curve', 'Tangent Through Origin', 'Optimal Point');

%ylim([-0.5, 0.5]);
hold off;


% Define constants
CD0 = 0.034;
K = 0.07;
CL_opt = 0.69693;  % Optimal lift coefficient from static optimization
CD_opt = CD0 + K * CL_opt^2;  % Optimal drag coefficient

rho = 1.225;  % Air density at sea level (kg/m^3)
S = 30;       % Reference area (m^2)
m = 1500;     % Aircraft mass (kg)
v0 = 100;     % Initial velocity (m/s)
x0 = 0;       % Initial horizontal position (m)
t_end = 100;  % Simulation end time (s)
dt = 0.1;     % Time step (s)

% Define the equation of motion
dxdt = @(v) v;
dvdt = @(v) -0.5 * rho * v^2 * S * CD_opt / m;

% Initialize variables
time = 0:dt:t_end;
v = zeros(size(time));
x = zeros(size(time));

% Initial conditions
v(1) = v0;
x(1) = x0;

% Simulate the dynamics over time
for i = 2:length(time)
    v(i) = v(i-1) + dvdt(v(i-1)) * dt;
    x(i) = x(i-1) + dxdt(v(i-1)) * dt;
end

% Plot the results
figure;
subplot(2, 1, 1);
plot(time, v, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Velocity over Time');

subplot(2, 1, 2);
plot(time, x, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Distance Traveled (m)');
title('Distance Traveled over Time');
grid on;

% Calculate the total distance traveled in the simulation
total_distance_sim = x(end);

% Display the total distance traveled
disp(['Total distance traveled in the simulation: ', num2str(total_distance_sim), ' meters']);

% Static optimization result for comparison (assuming static optimization yields a constant velocity)
v_static = v0;  % Assuming velocity is constant for static optimization
total_distance_static = v_static * t_end;

disp(['Total distance traveled in static optimization: ', num2str(total_distance_static), ' meters']);


