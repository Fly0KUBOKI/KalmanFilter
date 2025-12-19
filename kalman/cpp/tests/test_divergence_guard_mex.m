% Simple smoke test for mex_sensor_filter divergence guard
% Run from repository root: cd kalman/cpp/tests; test_divergence_guard_mex
try
    if exist('mex_sensor_filter','file')~=3
        error('mex_sensor_filter not found on path');
    end
    mex_sensor_filter('reset');
    mex_sensor_filter('log','off');
    innov = [1e9; 0; 0];              % huge innovation to trigger OUTLIER_SKIP
    dx_in = zeros(3,1);
    [dx_out, was_skipped, was_attenuated] = mex_sensor_filter('divergence_check','accel', innov, dx_in);
    fprintf('divergence_check returned was_skipped=%d was_attenuated=%d\n', logical(was_skipped), logical(was_attenuated));
    % Regularize covariance
    P = eye(15);
    Pout = mex_sensor_filter('divergence_regularize', P);
    fprintf('divergence_regularize returned %dx%d\n', size(Pout,1), size(Pout,2));
    fprintf('divergence_guard MEX smoke test PASSED\n');
catch ME
    fprintf('divergence_guard_mex test failed: %s\n', ME.message);
end
