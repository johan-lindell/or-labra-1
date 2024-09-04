
% ==================================================
% MS-E2132 - Laboratory Assignments in Operations Research II
% Optimal flight with a glider
% ==================================================

% State variables are x, h, vx and vh.

clear
close all

dp = 5;     %number of discretization points at the start
step = 3;   %increment of discretization points
dpend = 20; %number of discretization points at the end

sc = [400 50 13 5 30];    %scaling (x, y, vx, vh, t)

% initial and terminal conditions
x_0 = 0;
h_0 = 50;
h_f = 40;
vx_0 = 13;
vh_0 = 0;

% settings of the optimization algorithm
options = optimset(...
    'LargeScale', 'off',...
    'Display','iter',...
    'TolFun', 1d-3,...
    'TolX', 1d-3,...
    'TolCon', 1d-4,...
    'MaxFunEvals', 10000,...
    'Algorithm', 'sqp');

% initial guesses
x = linspace(0, 400, dp);
h = linspace(h_0, h_f, dp);
vx = vx_0 * ones(1, dp);
vh = zeros(1, dp);
cl = zeros(1, dp);
X = [x/sc(1) h/sc(2) vx/sc(3) vh/sc(4) cl 30/sc(5)];


% continuation loop
% =================

for iter = dp:step:dpend
    
    % upper bounds for variables
    ub_x = 1000 / sc(1) * ones(1, iter);
    ub_x(1) = x_0 / sc(1);
    
    ub_h = 90 / sc(2) * ones(1, iter);
    ub_h(1) = h_0 / sc(2);
    ub_h(iter) = h_f / sc(2);
    
    ub_vx = 40 / sc(3) * ones(1, iter);
    ub_vx(1) = vx_0 / sc(3);   
    %ub_vx(iter) = 9.2 / sc(3);
    
    ub_vh = 40 / sc(4) * ones(1, iter);
    ub_vh(1) = vh_0 / sc(4);
    %ub_vh(iter) = -3.9 / sc(4);
    
    ub_cl = 1.4 * ones(1, iter);
    
    ub_t = 100 / sc(5);
    
    ub=[ub_x ub_h ub_vx ub_vh ub_cl ub_t];
    
    % lower bounds for variables
    lb_x = ub_x(1) * ones(1, iter);
    
    lb_h = zeros(1, iter);
    lb_h(1) = ub_h(1);
    lb_h(iter) = ub_h(iter);
    
    lb_vx = 5 / sc(3) * ones(1, iter);
    lb_vx(1) = ub_vx(1);  
    lb_vx(iter) = 9.2/sc(3);  
    
    lb_vh = -10 / sc(4) * ones(1, iter);
    lb_vh(1) = ub_vh(1);
    %lb_vh(iter) = -3.9/sc(4);
    
    lb_cl(1:iter) = -1.4;  
    
    lb_t = 1/sc(5);
    
    lb = [lb_x lb_h lb_vx lb_vh lb_cl lb_t];
    
    % optimization
    [a, xf, exitflag, output, lambda] = fmincon('tobjfun', X,...
        [], [], [], [], lb, ub, 'tcollcon', options, iter, sc);
    
    % interpolation for new initial guesses
    origtime = linspace(0, a(end), iter);
    itime = linspace(0, a(end), iter + step);
    X_x = interp1(origtime, a(1:iter), itime, 'spline');
    X_h = interp1(origtime, a(iter+1:2*iter), itime, 'spline');
    X_vx = interp1(origtime, a(2*iter+1:3*iter), itime, 'spline');
    X_vh = interp1(origtime, a(3*iter+1:4*iter), itime, 'spline');
    cl = interp1(origtime, a(4*iter+1:5*iter), itime, 'linear');
    X = [X_x X_h X_vx X_vh cl a(end)];
    
    % plot the results
    figure(1)
    clf
    
    subplot(221)
    plot(a(1:iter)*sc(1) ,a(iter+1:2*iter)*sc(2), 'b-+')
    title('altitude vs. position')
    grid on
    xlabel('{\itx}, m')
    ylabel('{\ith}, m')
    
    subplot(222)
    plot(linspace(0, a(end), iter) * sc(5),...
        sqrt((a(2*iter+1:3*iter)*sc(3)).^2 + (a(3*iter+1:4*iter)*sc(4)).^2), 'b-+')
    grid on
    title('velocity vs. time')
    xlabel('{\itt}, s')
    ylabel('{\itv}, m/s')
    
    subplot(223)
    ka = (a(4*iter+1:5*iter-1) + a(4*iter+2:5*iter)) / 2;
    plot(a(end)*linspace(1/(iter-1)/2, 1-1/(iter-1)/2, iter-1)*sc(5), ka, 'b-+');
    grid on
    title('control');
    xlabel('{\itt}, s')
    ylabel('{\itC_L}')
    
    subplot(224)
    plot(linspace(0, a(end), iter)*sc(5), atan2(a(3*iter+1:4*iter)*sc(4),...
        a(2*iter+1:3*iter)*sc(3)),'b-+')
    grid on
    title('flight path angle vs. time')
    xlabel('{\itt}, s')
    ylabel('{\it\gamma}, rad')
    
    drawnow;
    
    fprintf('\nThe number of discretization points was %.0f\n',iter);
    fprintf('x(tf) = %.2f m\n',a(iter)*sc(1));
    fprintf('Distance travelled: x(tf) - x(0) = %.2f m\n',(a(iter)-a(1))*sc(1));
    fprintf('vx(tf) = %.2f m/s\n',a(3*iter)*sc(3));
    fprintf('Terminal time tf = %.2f s\n\n\n',a(end)*sc(5));
end
