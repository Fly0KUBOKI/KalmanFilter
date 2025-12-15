% compare_accel_filters.m
% Compare mex_sensor_filter('accel',...) vs MATLAB AccelFilter_cpp fallback
clear; clc;
addpath(fullfile(pwd, '..', '..', '..'));
% sample
a_sample = [0.1; -0.02; 9.7];
a_expected = [0;0;9.80665];
% call mex if available
mex_ok = (exist('mex_sensor_filter','file') == 3);
if mex_ok
    try
        [out_mex, is_out_mex] = mex_sensor_filter('accel', a_sample, a_expected);
        fprintf(' -> mex_sensor_filter -> [%.6g %.6g %.6g], outlier=%d\n', out_mex(1), out_mex(2), out_mex(3), is_out_mex);
    catch ME
        fprintf(' -> mex error: %s\n', ME.message);
        out_mex = [];
    end
else
    fprintf(' -> mex_sensor_filter not available\n');
    out_mex = [];
end

% MATLAB wrapper
try
    [out_matlab, is_out_matlab] = AccelFilter_cpp(a_sample, a_expected, 0.3, 20);
    fprintf(' -> MATLAB AccelFilter_cpp -> [%.6g %.6g %.6g], outlier=%d\n', out_matlab(1), out_matlab(2), out_matlab(3), is_out_matlab);
catch ME
    fprintf(' -> MATLAB AccelFilter_cpp error: %s\n', ME.message);
    out_matlab = [];
end

if ~isempty(out_mex) && ~isempty(out_matlab)
    diff = norm(out_mex - out_matlab);
    fprintf(' -> diff norm = %.3e\n', diff);
    if diff < 1e-6
        fprintf(' --> PASS: mex and MATLAB outputs match closely.\n');
    else
        fprintf(' --> WARN: mex vs MATLAB difference detected.\n');
    end
end
