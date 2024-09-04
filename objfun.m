function f = objfun(X, dp, sc)

% MS-E2132 - Laboratory Assignments in Operations Research II, assignment 1
% Optimal flight with a glider
% A function required by the Matlab-script 'fmincon', that returns the
% value of the function to be minimized at a given point.

% The function value is the x-coordinate at the end (fmincon minimizes)
f = -X(dp);