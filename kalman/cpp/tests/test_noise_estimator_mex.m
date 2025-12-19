% Simple smoke test for mex_sensor_filter noise estimator
% Run from repository root: cd kalman/cpp/tests; test_noise_estimator_mex
try
    if exist('mex_sensor_filter','file')~=3
        error('mex_sensor_filter not found on path');
    end
    mex_sensor_filter('reset');
    mex_sensor_filter('log','off');
    R_accel = mex_sensor_filter('get_R','accel');
    fprintf('get_R accel size: %dx%d\n', size(R_accel,1), size(R_accel,2));

    % Build a fake innovation, H, P and call noise_estimate
    innov = [0.1; 0.05; 0.02];
    H = eye(3);
    P = eye(15);
    mex_sensor_filter('noise_estimate','accel', innov, H, P);

    R_accel2 = mex_sensor_filter('get_R','accel');
    fprintf('R_accel before/after diag: %g / %g\n', diag(R_accel).' , diag(R_accel2).');
    fprintf('noise_estimate MEX smoke test PASSED\n');
catch ME
    fprintf('noise_estimator_mex test failed: %s\n', ME.message);
end
