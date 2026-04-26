% ---------------------------------------------------------
% MATLAB Startup Script
% ---------------------------------------------------------
disp(['Running startup.m: Pre-warming the Parallel Pool...(this may take' ...
    ' upto 30 seconds)']);

% Check if a pool already exists; if not, start 6 workers
p = gcp('nocreate');
if isempty(p)
    parpool(6);
end

disp('Parallel Pool Ready.');