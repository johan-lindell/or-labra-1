% Script to run simulations with different parameters

% First set of parameters
X0 = [0; 240; 12; 0];    % Initial state [x, h, v, gamma]
tspan = [0, 100];       % Time span [start, end]
m = 100;                % Mass in kg
cd0 = 0.034;            % Zero drag coefficient
k = 0.07;               % Induced drag coefficient
s = 14;                 % Reference area in m^2
g = 9.81;               % Gravitational acceleration in m/s^2
rho = 1.13;             % Air density in kg/m^3s
cl = 0;                 % Lift coefficient


% Vary air density (rho)
rho_array = [1.13, 0.5, 3.0];
simulate_flight(X0, tspan, m, cd0, k, s, g, rho_array, cl);

% Vary mass (m)
m_array = [100, 10, 300];
simulate_flight(X0, tspan, m_array, cd0, k, s, g, rho, cl);


% Vary gravitational constant (g)
g_array = [9.81, 15, 4.0];
simulate_flight(X0, tspan, m, cd0, k, s, g_array, rho, cl);

% Vary Reference area (s)
s_array = [14, 30, 4.0];
simulate_flight(X0, tspan, m, cd0, k, s_array, g, rho, cl);

% Vary zero drag coefficient (cd0)
cd0_array = [0.034, 1.5, 0];
simulate_flight(X0, tspan, m, cd0_array, k, s, g, rho, cl);

% Vary lift coefficient (cl)
%cl_array = [0.4, 0.5, 0.6];
%simulate_flight(X0, tspan, m, cd0, k, s, g, rho, cl_array);
