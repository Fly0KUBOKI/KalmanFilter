% test_ukf_gps.m - UKF based GPS update test

clear; clc;
addpath(genpath(pwd));

try
    % Load configuration
    fprintf('Loading configuration...\n');
    params = config_params();
    dt = params.dt;

    % Load sensor data
    fprintf('Loading sensor data...\n');
    obs = readtable('GenerateData/sensor_data.csv');
    fprintf('Loaded %d observations\n', height(obs));

    % Initialize ESKF
    fprintf('Initializing ESKF with static_time=%.2f, dt=%.4f...\n', params.static_time, dt);
    eskf = ESKF(obs, params.static_time, dt);
    fprintf('ESKF initialized successfully\n');
    fprintf('Initial position: [%.3f, %.3f, %.3f]\n', eskf.p(1), eskf.p(2), eskf.p(3));

    % Test a single GPS update
    fprintf('\nTesting single GPS update at step 100...\n');
    k = 100;
    
    % Check if GPS data is available
    if ~isnan(obs.gps_lat(k)) && ~isnan(obs.gps_lon(k))
        fprintf('GPS data at k=%d: lat=%.6f, lon=%.6f, alt=%.2f\n', k, obs.gps_lat(k), obs.gps_lon(k), obs.gps_alt(k));
        
        % Call update_filter
        eskf.update_filter(obs, k);
        fprintf('Update successful\n');
        fprintf('Updated position: [%.3f, %.3f, %.3f]\n', eskf.p(1), eskf.p(2), eskf.p(3));
    else
        fprintf('No GPS data at step %d\n', k);
    end

    fprintf('\nTest completed successfully!\n');
    
catch ME
    fprintf('\n=== ERROR OCCURRED ===\n');
    fprintf('Message: %s\n', ME.message);
    fprintf('\nStack trace:\n');
    for i = 1:length(ME.stack)
        fprintf('  [%d] %s (line %d)\n', i, ME.stack(i).name, ME.stack(i).line);
    end
    rethrow(ME);
end
