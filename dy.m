function Xdot = dy(X, cl)

% Test push
% MS-E2132 - Laboratory Assignments in Operations Research II, assignment 1
% State equations of the glider

% parameters
m = 100;	    % mass
cd0 = 0.034;    % zero drag coefficient
k = 0.07;	    % induced drag coefficient
s = 14;		    % reference area
g = 9.809;	    % gravitational acceleration
rho = 1.13;	    % air density 

% state variables
x = X(1);       % x-coordinate
h = X(2);       % altitude
v = X(3);       % velocity
gamma = X(4);   % flight path angle

% state equations
xdot = v * cos(gamma);      % Horizontal velocity
hdot = v * sin(gamma);      % Vertical velocity
vdot = -g * sin(gamma) - (cd0 + k * cl^2) * s * 0.5 * rho * v^2 / m;  % Change in velocity
gammadot = -g * cos(gamma) / v + cl * s * 0.5 * rho * v / m;          % Change in flight path angle

Xdot = [xdot; hdot; vdot; gammadot];

end
