function simulate_flight(X0, tspan, m, cd0, k, s, g, rho, cl)
    % This function simulates flight dynamics and plots the results.
    % It allows one parameter among m, cd0, k, s, g, rho, cl to be an array.
    % If a parameter is an array, it loops over its values, simulates with different parameters,
    % and plots these different simulations on the same plot.
    
    % Initialize figure and subplots
    figureHandle = figure;
    set(gcf, 'Position',  [100, 100, 700, 600])
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
    ylabel('gamma (rad)');
    
    % Prepare ODE options with event function
    options = odeset('Events', @myEventsFcn);
    
    % Determine which parameter is an array
    params = {'m', m; 'cd0', cd0; 'k', k; 's', s; 'g', g; 'rho', rho; 'cl', cl};
    array_params = {};
    array_param_name = '';
    array_param_values = [];
    
    for i = 1:size(params,1)
        param_value = params{i,2};
        if length(param_value) > 1
            array_params{end+1} = params{i,1};
            array_param_name = params{i,1};
            array_param_values = param_value;
        end
    end
    
    if length(array_params) > 1
        error('Only one parameter can be an array at a time.');
    elseif isempty(array_params)
        % No array parameters, run simulation once
        simulate_and_plot(m, cd0, k, s, g, rho, cl, []);
    else
        % Loop over the array parameter
        for idx = 1:length(array_param_values)
            % Set the parameter to its current value
            m_i = m;
            cd0_i = cd0;
            k_i = k;
            s_i = s;
            g_i = g;
            rho_i = rho;
            cl_i = cl;
            
            switch array_param_name
                case 'm'
                    m_i = array_param_values(idx);
                case 'cd0'
                    cd0_i = array_param_values(idx);
                case 'k'
                    k_i = array_param_values(idx);
                case 's'
                    s_i = array_param_values(idx);
                case 'g'
                    g_i = array_param_values(idx);
                case 'rho'
                    rho_i = array_param_values(idx);
                case 'cl'
                    cl_i = array_param_values(idx);
                otherwise
                    error('Unknown parameter name');
            end
            % Simulate and plot
            simulate_and_plot(m_i, cd0_i, k_i, s_i, g_i, rho_i, cl_i, array_param_values(idx));
        end
    end
    
    % Add legends to the plots
    subplot(3, 1, 1);
    legend show;
    subplot(3, 1, 2);
    legend show;
    subplot(3,1,3);
    legend show;
    
    % Function for the event (stopping condition)
    function [value, isterminal, direction] = myEventsFcn(t, X)
        value = X(2);       % Stop when altitude h = 0
        isterminal = 1;     % Stop the integration
        direction = -1;     % Negative direction only
    end
    
    % Function to simulate and plot
    function simulate_and_plot(m_i, cd0_i, k_i, s_i, g_i, rho_i, cl_i, param_value)
        % Solve the state equations using ode45
        [t, X] = ode45(@(t, X) dy_sim(X, cl_i, m_i, cd0_i, k_i, s_i, g_i, rho_i), tspan, X0, options);
        
        % Create label for this simulation
        if isempty(param_value)
            label_str = 'Simulation';
        else
            label_str = sprintf('%s = %.3f', array_param_name, param_value);
        end
        
        % Plot Flight Path
        subplot(3, 1, 1);
        plot(X(:,1), X(:,2), 'DisplayName', label_str);
        %ylim(0,max(X(:,1)))
        
        % Plot Velocity
        subplot(3, 1, 2);
        plot(t, X(:, 3), 'DisplayName', label_str);
        
        % Plot Flight Path Angle
        subplot(3,1,3);
        plot(t, X(:, 4), 'DisplayName', label_str);
    end
    
    % Add parameters to the figure title
    % Exclude the parameter being varied
    fixed_params_str = '';
    for i = 1:size(params,1)
        param_name = params{i,1};
        param_value = params{i,2};
        if ~strcmp(param_name, array_param_name)
            fixed_params_str = [fixed_params_str, sprintf('%s = %.3f, ', param_name, param_value)];
        end
    end
    % Remove trailing comma and space
    if ~isempty(fixed_params_str)
        fixed_params_str = fixed_params_str(1:end-2);
    end
    
    sgtitle(sprintf('%s\nx_0 = %.2f m, h_0 = %.2f m, v_0 = %.2f m/s, \\gamma_0 = %.2f rad', fixed_params_str, X0(1), X0(2), X0(3), X0(4)));

    %% Save the figure to './figures' directory in PNG format
    % Create 'figures' directory if it doesn't exist
    figures_dir = './figures';
    if ~exist(figures_dir, 'dir')
        mkdir(figures_dir);
    end
    
    % Generate filename based on parameter being varied
    if isempty(array_param_name)
        filename = 'simulation';
    else
        filename = sprintf('simulation_%s', array_param_name);
    end
    % Replace any invalid characters in filename
    filename = regexprep(filename, '[^\w\-]', '_');
    
    % Full path to save the figure
    filepath = fullfile(figures_dir, [filename, '.png']);
    
    % Save the figure
    saveas(figureHandle, filepath);
    fprintf('Figure saved to %s\n', filepath);
    
    % Optional: Close the figure if you don't need it open
    % close(figureHandle);
end
