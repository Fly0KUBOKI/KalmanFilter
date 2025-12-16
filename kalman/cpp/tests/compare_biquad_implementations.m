% compare_biquad_implementations.m
% Compare outputs of: mex_sensor_filter('gyro',...), MATLAB BiquadFilter, and C++-style Biquad implemented in MATLAB

repo_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
obs = readtable(fullfile(repo_root,'GenerateData','sensor_data.csv'));
N = 200; % samples to compare
dt = 1/200;
cutoff = 30;

% Prepare MATLAB Biquad objects (per-axis)
mat_filters = cell(3,1);
for i=1:3
    mat_filters{i} = BiquadFilter(1/dt, cutoff); % sample_rate, cutoff
end

% C++-style biquad state containers
cpp_state = struct();
cpp_state.x1 = zeros(3,1);
cpp_state.x2 = zeros(3,1);
cpp_state.y1 = zeros(3,1);
cpp_state.y2 = zeros(3,1);
cpp_state.coeffs = struct('b0',0,'b1',0,'b2',0,'a1',0,'a2',0);
cpp_state.coeffs_set = false;

function coeffs = compute_cpp_coeffs(dt, cutoff)
    sample_rate = 1.0/dt;
    omega = 2.0*pi*cutoff / sample_rate;
    K = tan(omega/2.0);
    Q = 1.0 / sqrt(2.0);
    norm = 1.0 / (1.0 + K / Q + K * K);
    b0 = K * K * norm;
    b1 = 2.0 * b0;
    b2 = b0;
    a1 = 2.0 * (K * K - 1.0) * norm;
    a2 = (1.0 - K / Q + K * K) * norm;
    coeffs = struct('b0',b0,'b1',b1,'b2',b2,'a1',a1,'a2',a2);
end

% helper to apply C++-style biquad to a scalar input per axis
function [y, state] = cpp_biquad_apply(x, state, coeffs)
    % x: scalar input
    % state: struct with x1,x2,y1,y2 scalars
    w = x - coeffs.a1 * state.y1 - coeffs.a2 * state.y2;
    y = coeffs.b0 * w + coeffs.b1 * state.x1 + coeffs.b2 * state.x2;
    % update
    state.x2 = state.x1;
    state.x1 = w;
    state.y2 = state.y1;
    state.y1 = y;
end

% Precompute coeffs
coeffs = compute_cpp_coeffs(dt, cutoff);

% allocate
mat_out = zeros(N,3);
cpp_out = zeros(N,3);

for k=1:N
    w = [obs.gyro_x(k); obs.gyro_y(k); obs.gyro_z(k)];
    % (MEX gyro removed) — do not call mex_sensor_filter('gyro',...)
    % MATLAB Biquad per-axis
    for ax=1:3
        mat_out(k,ax) = mat_filters{ax}.apply(w(ax));
    end
    % C++-style: apply per-axis with shared coeffs but independent state
    for ax=1:3
        s = struct('x1',cpp_state.x1(ax),'x2',cpp_state.x2(ax),'y1',cpp_state.y1(ax),'y2',cpp_state.y2(ax));
        [y, s] = cpp_biquad_apply(w(ax), s, coeffs);
        cpp_state.x1(ax)=s.x1; cpp_state.x2(ax)=s.x2; cpp_state.y1(ax)=s.y1; cpp_state.y2(ax)=s.y2;
        cpp_out(k,ax) = y;
    end
end

% Summaries (compare MATLAB vs C++-style)
fprintf('RMSE matlab vs cppstyle per-axis: %g %g %g\n', sqrt(mean((mat_out - cpp_out).^2,1)) );

% Print first 10 samples
for k=1:10
    fprintf('k=%d time=%.4f | mat: %g %g %g | cppstyle: %g %g %g\n', k, obs.time(k), mat_out(k,1),mat_out(k,2),mat_out(k,3), cpp_out(k,1),cpp_out(k,2),cpp_out(k,3));
end
