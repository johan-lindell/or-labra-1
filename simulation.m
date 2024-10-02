% Script to run simulations with different parameters

% First set of parameters
X0 = [0; 240; 2; 0];    % Initial state [x, h, v, gamma]
tspan = [0, 100];       % Time span [start, end]
m = 100;                % Mass in kg
cd0 = 0.034;            % Zero drag coefficient
k = 0.07;               % Induced drag coefficient
s = 14;                 % Reference area in m^2
g = 9.81;               % Gravitational acceleration in m/s^2
rho = 1.13;             % Air density in kg/m^3s
cl = 0.6969;                 % Lift coefficient

simulate_flight(X0, tspan, m, cd0, k, s, g, rho, cl);

%rho_1 = 0.5;
%simulate_flight(X0, tspan, m, cd0, k, s, g, rho_1, cl);
%
%rho_2 = 2;
%simulate_flight(X0, tspan, m, cd0, k, s, g, rho_2, cl);
