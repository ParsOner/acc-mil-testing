function resultsLHS = RUN_ACC_LHS(N)

    % If N is not provided, use a default value
    if nargin < 1
        N = 210;
    end

    % Name of the Simulink model
    model = 'CCToACCFinal1';  

    % Check that the model is loaded before running the simulations
    if ~bdIsLoaded(model)
        error('Model %s is not loaded. Open it before running RUN_ACC_LHS.', model);
    end

    % --- Parameter ranges (min/max values for LHS sampling) ---
    % Speed setpoint [m/s]
    v_min = 15;    v_max = 35;
    % Time gap [s]
    tg_min = 0.9;  tg_max = 2.3;
    % Default spacing [m]
    d_min = 2.5;   d_max = 7;

    % Vehicle mass [kg]
    m_min = 900;   m_max = 1600;

    % Combined aerodynamic term Cd * Af  (min and max)
    Cd_air_min = 0.25*1.8;   % min Cd * min Af
    Cd_air_max = 0.35*2.3;   % max Cd * max Af

    % Rolling resistance coefficient [-]
    Cr_min = 0.007; Cr_max = 0.016;

    % Road slope angle theta [rad]
    th_min = -0.035; th_max = 0.07;

    % --- Latin Hypercube Sampling (7-dimensional) ---
    % X is an N-by-7 matrix with values in [0,1] for each dimension.
    X = lhsdesign(N, 7);

    % Scale LHS values to the physical ranges of each parameter
    v_set_vals      = v_min  + X(:,1)*(v_max  - v_min);
    time_gap_vals   = tg_min + X(:,2)*(tg_max - tg_min);
    d_default_vals  = d_min  + X(:,3)*(d_max  - d_min);
    
    m_vals          = m_min  + X(:,4)*(m_max - m_min);
    Cd_air_vals     = Cd_air_min + X(:,5)*(Cd_air_max - Cd_air_min);
    Cr_vals         = Cr_min + X(:,6)*(Cr_max - Cr_min);
    theta_vals      = th_min + X(:,7)*(th_max - th_min);
    % X(:,8) is currently unused, but reserved for a possible extra parameter.

    % --- Pre-allocate output struct (for speed and clean code) ---
    resultsLHS(N,1) = struct( ...
        'v_set',[], 'time_gap',[], 'default_spacing',[], ...
        'm',[], 'Cd_air',[], 'theta',[], ...
        'TTC_min',[], 'viol_rate',[], 'a_max',[], 'a_min',[] );

    % --- Enable Fast Restart for faster multiple simulations ---
    set_param(model, 'FastRestart', 'on');

    % --- Main simulation loop ---
    for i = 1:N

        % Send sampled parameters to the base workspace
        assignin('base','v_set',           v_set_vals(i));
        assignin('base','time_gap',        time_gap_vals(i));
        assignin('base','default_spacing', d_default_vals(i));
        assignin('base','m',               m_vals(i));
        assignin('base','Cd_air',          Cd_air_vals(i));
        assignin('base','Cr',              Cr_vals(i));
        assignin('base','theta',           theta_vals(i));

        % Run Simulink model.
        % The option 'ReturnWorkspaceOutputs' sends logged signals to simOut.
        simOut = sim(model, ...
            'StopTime','40', ...
            'ReturnWorkspaceOutputs','on');

        % --- Read KPI signals from simOut ---
        TTC_min_ts   = simOut.TTC_min;
        viol_ts      = simOut.viol_rate;
        a_max_ts     = simOut.a_max;
        a_min_ts     = simOut.a_min;

        % If the signal is a timeseries object, read data from the .Data field.
        % Otherwise, assume it is a numeric vector and take the last element.
        if isa(a_max_ts,'timeseries')
            a_max_val = max(a_max_ts.Data);     % Maximum acceleration during the run
        else
            a_max_val = a_max_ts(end);
        end

        if isa(a_min_ts,'timeseries')
            a_min_val = min(a_min_ts.Data);     % Minimum acceleration (strongest braking)
        else
            a_min_val = a_min_ts(end);
        end

        % --- Store input parameters in the struct ---
        resultsLHS(i).v_set           = v_set_vals(i);
        resultsLHS(i).time_gap        = time_gap_vals(i);
        resultsLHS(i).default_spacing = d_default_vals(i);
        resultsLHS(i).m               = m_vals(i);
        resultsLHS(i).Cd_air          = Cd_air_vals(i);
        resultsLHS(i).Cr              = Cr_vals(i);
        resultsLHS(i).theta           = theta_vals(i);

        % --- Store KPI values in the struct ---
        % ---- TTC_min ----
        if isa(TTC_min_ts,'timeseries')
            % Use the last value of TTC_min at the end of the simulation
            resultsLHS(i).TTC_min = TTC_min_ts.Data(end);
        else
            resultsLHS(i).TTC_min = TTC_min_ts(end);
        end

        % ---- viol_rate ----
        if isa(viol_ts,'timeseries')
            % Use the last value of viol_rate at the end of the simulation
            resultsLHS(i).viol_rate = viol_ts.Data(end);
        else
            resultsLHS(i).viol_rate = viol_ts(end);
        end

        % Store a_max and a_min (already reduced to scalar values above)
        resultsLHS(i).a_max = a_max_val;
        resultsLHS(i).a_min = a_min_val;

        fprintf('Sim %3d/%3d completed.\n', i, N);
    end

    % --- Disable Fast Restart after the loop ---
    set_param(model, 'FastRestart', 'off');

    % --- Export results to base workspace and display as a table ---
    T = struct2table(resultsLHS);
    disp(T);
    assignin('base','resultsLHS', resultsLHS);

end
