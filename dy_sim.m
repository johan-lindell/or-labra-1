function Xdot = dy_sim(X, cl, m, cd0, k, s, g, rho)

% Test push
% MS-E2132 - Laboratory Assignments in Operations Research II, assignment 1
% State equations of the glider

% state variables
%x = X(1);       % x-coordinate
%h = X(2);       % altitude
v = X(3);       % velocity
gamma = X(4);   % flight path angle

% state equations
xdot = v * cos(gamma);      % Horizontal velocity
hdot = v * sin(gamma);      % Vertical velocity
vdot = -g * sin(gamma) - (cd0 + k * cl^2) * s * 0.5 * rho * v^2 / m;  % Change in velocity
gammadot = -g * cos(gamma) / v + cl * s * 0.5 * rho * v / m;          % Change in flight path angle

Xdot = [xdot; hdot; vdot; gammadot];

end
