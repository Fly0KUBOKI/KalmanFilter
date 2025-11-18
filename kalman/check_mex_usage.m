% check_mex_usage.m
% MEXファイルの使用状況を確認

fprintf('=== MEX Usage Check ===\n\n');

% 1. MEXファイルの存在確認
fprintf('1. Checking MEX files...\n');
mex_file = 'mex_kalman_filter_core';
if exist(mex_file, 'file') == 3
    fprintf('   ✓ %s found: %s\n', mex_file, which(mex_file));
else
    fprintf('   ✗ %s NOT found\n', mex_file);
    fprintf('   Run: cd cpp; build_mex()\n');
end

% 2. kalman_filter_coreの使用確認
fprintf('\n2. Testing kalman_filter_core...\n');
P = eye(15) * 0.1;
q = [1; 0; 0; 0];
a_meas = [0; 0; 9.81];
ba = [0.01; -0.02; 0.03];
w_meas = [0.001; -0.001; 0.0005];
bg = [0.0001; 0.0001; -0.0001];
Q = eye(15) * 1e-5;
dt = 0.01;

try
    tic;
    P_out = kalman_filter_core('predict_step', P, q, a_meas, ba, w_meas, bg, Q, dt);
    t_elapsed = toc;
    fprintf('   ✓ kalman_filter_core works (%.6f ms)\n', t_elapsed * 1000);
    fprintf('   Output size: %dx%d\n', size(P_out, 1), size(P_out, 2));
catch ME
    fprintf('   ✗ Error: %s\n', ME.message);
end

% 3. パフォーマンス比較
fprintf('\n3. Performance test (100 iterations)...\n');
n_iter = 100;

% Clear persistent variable to force re-detection
clear kalman_filter_core;

tic;
for i = 1:n_iter
    P_test = kalman_filter_core('predict_step', P, q, a_meas, ba, w_meas, bg, Q, dt);
end
t_total = toc;

fprintf('   Average time: %.6f ms per call\n', (t_total / n_iter) * 1000);
fprintf('   Total time: %.3f seconds\n', t_total);

% 4. 直接MEX呼び出しテスト
fprintf('\n4. Direct MEX call test...\n');
if exist(mex_file, 'file') == 3
    try
        tic;
        P_mex = mex_kalman_filter_core('predict_step', P, q, a_meas, ba, w_meas, bg, Q, dt);
        t_mex = toc;
        fprintf('   ✓ Direct MEX call works (%.6f ms)\n', t_mex * 1000);
        
        % 結果の差分チェック
        diff_norm = norm(P_test - P_mex, 'fro');
        fprintf('   Difference from MATLAB wrapper: %.2e\n', diff_norm);
    catch ME
        fprintf('   ✗ MEX Error: %s\n', ME.message);
    end
else
    fprintf('   ✗ MEX file not available\n');
end

fprintf('\n=== Check Complete ===\n');
