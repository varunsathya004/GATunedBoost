function [Kp, Ki] = ga_pid_tuner()
    clc;
    
    % 1. Check for existing pool
    p = gcp('nocreate');
    if isempty(p)
        p = parpool(6);
    elseif p.NumWorkers < 6
        delete(p);
        p = parpool(6);
    end
    
    %% 2. Parameters (Fixed for all runs)
    Vin = 12; Vout_ref = 20; L = 76.8e-6; C = 400e-6; R = 4; beta = 1;
    D_prime = Vin / Vout_ref;
    
    % Raw continuous polynomial arrays for the Plant
    P_num = (Vin / (D_prime^2)) * [-L, R*(D_prime^2)] * beta;
    P_den = [(L * C * R), L, R*(D_prime^2)];
    
    %% THE SPEED HACK: Discretize the Plant ONCE before the GA runs
    dt = 1e-4; % Sample time
    t_eval = 0:dt:0.01;
    step_input = Vout_ref * ones(1, length(t_eval)); % Pre-allocate input array
    
    Plant_s = tf(P_num, P_den);
    Plant_d = c2d(Plant_s, dt, 'tustin'); % Convert to discrete z-domain
    [b_plant, a_plant] = tfdata(Plant_d, 'v'); % Extract raw z-domain arrays
    
    %% 3. Parallel Multi-Run Setup
    num_runs = 10;
    all_K = zeros(num_runs, 2);
    all_fval = zeros(num_runs, 1);
    
    options = optimoptions('ga', ...
        'PopulationSize', 50, ...
        'MaxGenerations', 100, ...
        'MaxStallGenerations', 50, ...
        'UseParallel', false, ... 
        'Display', 'none');
        
    parfor i = 1:num_runs
        fprintf('Starting GA Session %d on worker...\n', i);
        % Pass the pre-calculated z-domain arrays and step input
        cost_func = @(K) evaluate_boost_pi_discrete(K, b_plant, a_plant, Vout_ref, step_input, dt);
        
        [sol, fval] = ga(cost_func, 2, [], [], [], [], [0.001, 50], [50, 400], [], options);
        
        all_K(i, :) = sol;
        all_fval(i) = fval;
    end
    
    %% 4. Pick the Result with the Least Cost
    [min_cost, best_idx] = min(all_fval); 
    
    Kp_final = all_K(best_idx, 1);
    Ki_final = all_K(best_idx, 2);
    
    fprintf('\n--- Best Result Found (Run %d) ---\n', best_idx);
    fprintf('Least Cost: %.4f\n', min_cost);
    fprintf('Optimal Kp: %.6f\n', Kp_final);
    fprintf('Optimal Ki: %.6f\n', Ki_final);
    
    Kp = Kp_final;
    Ki = Ki_final;
    
    %% 5. Verification Plot using Continuous Objects
    sys_cl = feedback((pid(Kp_final, Ki_final) * (1/20)) * Plant_s, 1);
    
    if ~isstable(sys_cl)
        warning('The optimized controller resulted in an UNSTABLE system!');
    end
    % Creates a new window with a custom name and removes the "Figure X" prefix
    figure('Name', 'GA Controller Verification', 'NumberTitle', 'off');
    step(Vout_ref * sys_cl, 0.3); grid on;
    title(sprintf('Best Response (Run %d) | Cost: %.2f | Kp: %.2f, Ki: %.2f', best_idx, min_cost, Kp_final, Ki_final));
end

%% Z-DOMAIN COST FUNCTION (Blazing Fast)
function cost = evaluate_boost_pi_discrete(K, b_plant, a_plant, Vref, step_input, dt)
    % Factor in the (1/20) scaling from your original code
    Kp_eff = K(1) / 20; 
    Ki_eff = K(2) / 20;
    
    % Tustin Discretization of the PI Controller: C(z) = (c0 + c1*z^-1) / (1 - z^-1)
    c0 = Kp_eff + Ki_eff * (dt / 2);
    c1 = -Kp_eff + Ki_eff * (dt / 2);
    
    % Convolve arrays to get Open Loop L(z) = C(z) * P(z)
    L_num = conv([c0, c1], b_plant);
    L_den = conv([1, -1], a_plant);
    
    % Closed Loop T(z) = L(z) / (1 + L(z))
    T_num = L_num;
    T_den = L_den + L_num;
    
    % SIMULATE using pre-compiled C-backend (filter)
    y = filter(T_num, T_den, step_input);
    
    % FAST STABILITY CHECK: If the output blows up to infinity or NaN, it's unstable
    if any(abs(y) > 10 * Vref) || any(isnan(y))
        cost = 1e8; 
        return; 
    end
    
    % Cost Calculation (Vectorized)
    steady_state_error = abs(Vref - y(end));
    overshoot = max(0, max(y) - Vref);
    
    % Time array matching the output length
    t = (0:length(y)-1) * dt;
    itae = sum(t .* abs(Vref - y));
    
    cost = 100 * steady_state_error + 10 * overshoot + itae;
end
