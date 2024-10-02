X0 = [0; 240; 13; 0];    % Initial state [x, h, v, gamma]
tspan = [0, 200];       % Time span [start, end]
m = 100;                % Mass in kg
cd0 = 0.034;            % Zero drag coefficient
k = 0.07;               % Induced drag coefficient
s = 14;                 % Reference area in m^2
g = 9.81;               % Gravitational acceleration in m/s^2
rho = 1.13;             % Air density in kg/m^3s
cl = 0.6969;  
% cl - Lift coefficient


%options = odeset('Events', @myEventsFcn);

% Solve the state equations using ode45
[t, X] = ode45(@(t, X) dy_sim(X, cl, m, cd0, k, s, g, rho), tspan, X0);

% Plot the results
figure;
plot(X(:,1), X(:,2));
title('Flight simulation')
xlabel('x');
ylabel('h');
ylim([0,250]);
xlim([0,2700])