% run_all_sensor_filter_tests.m
% 全 MEX センサーフィルタ関連コマンドの単体・統合テスト

fprintf('Running sensor_filter MEX tests...\n');

% Build MEX (best-effort)
try
    oldpwd = pwd;
    % compute repository root and build/bin directories robustly
    script_path = mfilename('fullpath');
    script_dir = fileparts(script_path);
    build_dir = '';
    bin_dir = '';
    repo_root = '';
    cur = script_dir;
    % search upward up to 6 levels for expected build path
    for k = 0:6
        if exist(fullfile(cur, 'kalman', 'cpp', 'build'), 'dir')
            repo_root = cur;
            build_dir = fullfile(cur, 'kalman', 'cpp', 'build');
            bin_dir = fullfile(cur, 'kalman', 'cpp', 'bin');
            break;
        end
        if exist(fullfile(cur, 'cpp', 'build'), 'dir')
            % cur likely points to kalman; set repo_root to parent
            repo_root = fileparts(cur);
            build_dir = fullfile(cur, 'cpp', 'build');
            bin_dir = fullfile(cur, 'cpp', 'bin');
            break;
        end
        parent = fileparts(cur);
        if strcmp(parent, cur)
            break;
        end
        cur = parent;
    end
    if isempty(build_dir)
        % fallback to original heuristic
        repo_root = fileparts(fileparts(fileparts(script_path)));
        build_dir = fullfile(repo_root, 'kalman', 'cpp', 'build');
        bin_dir = fullfile(repo_root, 'kalman', 'cpp', 'bin');
    end
    cd(build_dir);
    if exist('build_mex','file') == 2 || exist('build_mex','file') == 6
        fprintf('Building MEX (build_mex)...\n');
        build_mex();
    else
        fprintf('build_mex not found; attempting mex build_sensor_filter.m if present.\n');
        if exist('build_sensor_filter.m','file') == 2
            build_sensor_filter;
        end
    end
    % ensure MEX binaries are on path
    if exist(bin_dir,'dir')==7
        addpath(bin_dir);
    end
    cd(oldpwd);
catch ME
    fprintf('Build step failed or skipped: %s\n', ME.message);
end

clear mex

tol = 1e-6;
all_ok = true;

%% Test A: get_R vs MATLAB NoiseEstimator
fprintf('\nTest A: get_R vs MATLAB NoiseEstimator\n');
try
    ne = NoiseEstimator(5);
    R_mat = ne.getRnoise('accel');
    R_mex = mex_sensor_filter('get_R','accel');
    d = norm(R_mat - R_mex,'fro');
    fprintf('||R_mat - R_mex||_F = %g\n', d);
    if d < 1e-6
        fprintf('PASS: get_R matches MATLAB NoiseEstimator\n');
    else
        fprintf('WARN: get_R differs (norm=%g)\n', d);
        all_ok = false;
    end
catch ME
    fprintf('ERROR Test A: %s\n', ME.message);
    all_ok = false;
end

%% Test B: noise_estimate updates R
fprintf('\nTest B: noise_estimate updates R\n');
try
    R_before = mex_sensor_filter('get_R','accel');
    innov = [0.2; 0.0; 0.0];
    H = eye(3);
    Ppred = eye(3)*0.01;
    mex_sensor_filter('noise_estimate','accel', innov, H, Ppred);
    R_after = mex_sensor_filter('get_R','accel');
    change = norm(R_after - R_before,'fro');
    fprintf('Change in R (Fro): %g\n', change);
    if change > 0
        fprintf('PASS: noise_estimate changed R\n');
    else
        fprintf('WARN: noise_estimate did not change R\n');
        all_ok = false;
    end
catch ME
    fprintf('ERROR Test B: %s\n', ME.message);
    all_ok = false;
end

%% Test C: divergence_check
fprintf('\nTest C: divergence_check\n');
try
    % Prepare a dx and large innovation for gps (3-dim)
    dx_in = zeros(15,1);
    dx_in(1) = 0.1;
    innov = [1000; 0; 0]; % large innovation to exercise attenuation/skip logic
    [dx_out, should_skip, was_attenuated] = mex_sensor_filter('divergence_check','gps', innov, dx_in);
    fprintf('dx_out size: %dx%d, should_skip=%d, was_attenuated=%d\n', size(dx_out,1), size(dx_out,2), logical(should_skip), logical(was_attenuated));
    if ismatrix(dx_out) && size(dx_out,1) == 15
        fprintf('PASS: divergence_check returned dx_out shape OK\n');
    else
        fprintf('FAIL: divergence_check dx_out unexpected shape\n');
        all_ok = false;
    end
catch ME
    fprintf('ERROR Test C: %s\n', ME.message);
    all_ok = false;
end

%% Test D: divergence_regularize
fprintf('\nTest D: divergence_regularize\n');
try
    P = eye(15) * 1e-12; % poorly conditioned
    Preg = mex_sensor_filter('divergence_regularize', P);
    r_before = rcond(P);
    r_after = rcond(Preg);
    fprintf('rcond before=%g, after=%g\n', r_before, r_after);
    if r_after > r_before || isnan(r_before)
        fprintf('PASS: divergence_regularize improved conditioning\n');
    else
        fprintf('WARN: divergence_regularize did not improve conditioning\n');
        all_ok = false;
    end
catch ME
    fprintf('ERROR Test D: %s\n', ME.message);
    all_ok = false;
end

fprintf('\nAll tests completed. Summary: %s\n', ternary(all_ok, 'ALL PASSED', 'SOME FAILURES'));

function s = ternary(cond, a, b)
    if cond, s = a; else s = b; end
end
