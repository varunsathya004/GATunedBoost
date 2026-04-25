function [Kp, Ki] = ga_pid_tuner()

clear; clc;
rng(42);

%% 1. Parameters
Vin = 12;
Vout_ref = 20;
L = 76.8e-6;
C = 400e-6;
R = 4;
beta = 1;
D_prime = Vin / Vout_ref;

%% 2. Plant Transfer Function
num = (Vin / (D_prime^2)) * [-L, R*(D_prime^2)];
den = [(L * C * R), L, R*(D_prime^2)];
G_vd = tf(num, den);
Plant = G_vd * beta;

%% 3. GA Setup
num_vars = 2;
lb = [0.001, 10];
ub = [1, 1000];

options = optimoptions('ga', ...
    'PopulationSize', 30, ...
    'MaxGenerations', 100, ...
    'MaxStallGenerations', 20, ...
    'FunctionTolerance', 1e-4, ...
    'Display', 'iter', ...
    'UseParallel', false);

cost_func = @(K) evaluate_boost_pi(K, Plant, Vout_ref);
[optimal_K, ~] = ga(cost_func, num_vars, [], [], [], [], lb, ub, [], options);

Kp_final = optimal_K(1);
Ki_final = optimal_K(2);
fprintf('\nGA Results: Kp = %.6f, Ki = %.6f\n', Kp_final, Ki_final);

%% 4. Verification Plot
% Apply the 1/20 gain to the verification plot as well
sys_cl = feedback((pid(Kp_final, Ki_final) * (1/20)) * Plant, 1);

figure('Name', 'GA PID Tuning Result', 'NumberTitle', 'off');
step(Vout_ref * sys_cl, 0.05); grid on;
yline(Vout_ref, 'r--', '20V Reference');
title(['GA Optimized Response | Kp: ', num2str(Kp_final), ', Ki: ', num2str(Ki_final)]);
ylabel('Output Voltage (V)');

%% Cost Function
function cost = evaluate_boost_pi(K, Plant, Vref)
    Kp = K(1); 
    Ki = K(2);
    
    % EMBEDDED GAIN: Multiply by 1/20 to match the Simulink model's error path
    C_ctrl = pid(Kp, Ki) * (1/20);
    sys_cl = feedback(C_ctrl * Plant, 1);

    % Stability check
    if ~isstable(sys_cl)
        cost = 1e8;
        return;
    end

    % Step response scaled to Vref
    t = 0:1e-4:0.01;
    y = Vref * step(sys_cl, t);

    % Voltage based penalties
    steady_state_error = abs(Vref - y(end));
    overshoot = max(0, max(y) - Vref);

    % ITAE
    error = abs(Vref - y);
    itae = sum(t .* error') * (t(2) - t(1));

    cost = 100 * steady_state_error + 10 * overshoot + itae;
end

Kp = Kp_final;
Ki = Ki_final;

end
