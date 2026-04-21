function [Kp_out, Ki_out] = ga_pid_tuner()
    % =====================================================================
    % Parallel Pooled Non-Linear Genetic Algorithm PID Tuner
    % Called extrinsically by Simulink at t=0
    % =====================================================================
    rng(42);
    disp('Simulink paused at t=0. Booting 10 parallel GA workers...');

    num_runs = 10; 
    best_Kp = zeros(num_runs, 1);
    best_Ki = zeros(num_runs, 1);
    best_costs = zeros(num_runs, 1);

    % Plant Parameters & Bounds
    Vin = 12; Vref = 20; L = 76.8e-6; C = 400e-6; R = 4;
    lb = [0.0001, 1.0]; 
    ub = [1, 100.0];

    % Options (UseParallel MUST be false here for parfor to work)
    options = optimoptions('ga', ...
        'Display', 'off', ...
        'PopulationSize', 30, ...
        'MaxGenerations', 50, ...
        'UseParallel', false); 
        
    % Execute 10 independent runs in parallel
    parfor i = 1:num_runs
        [opt_K, fval] = ga(@(K) eval_boost_ode(K, Vin, Vref, L, C, R), ...
                        2, [], [], [], [], lb, ub, [], options);
        best_Kp(i) = opt_K(1);
        best_Ki(i) = opt_K(2);
        best_costs(i) = fval;
        fprintf('Worker %d finished | Cost: %.4f | Kp: %.4f | Ki: %.4f\n', i, fval, opt_K(1), opt_K(2));
    end

    % Extract the absolute best from the 8 runs
    [absolute_best_cost, best_idx] = min(best_costs);
    Kp_out = best_Kp(best_idx);
    Ki_out = best_Ki(best_idx);

    fprintf('--- GA Complete. Resuming Simulink ---\n');
    fprintf('Global Champion (Worker %d): Kp = %f | Ki = %f (Cost: %f)\n', ...
            best_idx, Kp_out, Ki_out, absolute_best_cost);
end

% =========================================================================
% Cost Function
% =========================================================================
function cost = eval_boost_ode(K, Vin, Vref, L, C, R)
    Kp = K(1); Ki = K(2);
    x0 = [0; 0; 0];
    tspan = [0, 0.05]; 
    
    try
        warning('off', 'all');
        [t, x] = ode45(@(t, x) boost_dynamics(t, x, Kp, Ki, Vin, Vref, L, C, R), tspan, x0);
        warning('on', 'all');
        
        Vout = x(:, 2);
        error = abs(Vref - Vout); 
        
        if any(isnan(Vout)) || max(Vout) > 40 || Vout(end) < 0
            cost = 1e6;
            return;
        end
        
        % Integral Square Error
        dt = [0; diff(t)];
        cost = sum((error.^2) .* dt);
    catch
        cost = 1e6; 
    end
end

% =========================================================================
% Plant Dynamics: Soft-Start + Anti-Windup(clamped) + Damping
% =========================================================================
function dxdt = boost_dynamics(t, x, Kp, Ki, Vin, Vref, L, C, R)
    iL = x(1); Vout = x(2); err_int = x(3);
    
    Vref_dynamic = Vref * (1 - exp(-t / 0.005)); 
    error = Vref_dynamic - Vout;
    d = Kp * error + Ki * err_int;
    derr_int_dt = error;
    
    if d >= 0.85
        d = 0.85;
        if error > 0, derr_int_dt = 0; end
    elseif d <= 0.0
        d = 0.0;
        if error < 0, derr_int_dt = 0; end
    end
    
    RL = 0.05; 
    dVout_dt = ((1 - d) * iL) / C - Vout / (R * C);
    diL_dt = (Vin - iL * RL - (1 - d) * Vout) / L;
    
    dxdt = [diL_dt; dVout_dt; derr_int_dt];
end
