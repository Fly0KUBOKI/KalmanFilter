% debug_gyro.m - ジャイロフィルタのデバッグ

addpath('Common/Sensor');
addpath('cpp/bin');

% パラメータ
dt = 0.01;
cutoff = 20;

% MATLAB版の係数計算
sample_rate = 1.0 / dt;
omega = 2 * pi * cutoff / sample_rate;
K = tan(omega / 2);
Q = 1 / sqrt(2);
norm_val = 1 + K/Q + K^2;

b0 = K^2 / norm_val;
b1 = 2 * b0;
b2 = b0;
a1 = 2 * (K^2 - 1) / norm_val;
a2 = (1 - K/Q + K^2) / norm_val;

fprintf('MATLAB Biquad Coefficients:\n');
fprintf('  sample_rate = %.6f\n', sample_rate);
fprintf('  omega = %.6f\n', omega);
fprintf('  K = %.6f\n', K);
fprintf('  b0 = %.12f\n', b0);
fprintf('  b1 = %.12f\n', b1);
fprintf('  b2 = %.12f\n', b2);
fprintf('  a1 = %.12f\n', a1);
fprintf('  a2 = %.12f\n', a2);

% テストデータ
test_input = [1.0; 2.0; 3.0];

% MATLAB実装
matlab_filter = SensorFilterLib();
matlab_out1 = matlab_filter.filter_gyro(test_input, dt, cutoff);
matlab_out2 = matlab_filter.filter_gyro(test_input, dt, cutoff);
matlab_out3 = matlab_filter.filter_gyro(test_input, dt, cutoff);

fprintf('\nMATLAB Output (3 iterations):\n');
fprintf('  Out1 = [%.6f, %.6f, %.6f]\n', matlab_out1);
fprintf('  Out2 = [%.6f, %.6f, %.6f]\n', matlab_out2);
fprintf('  Out3 = [%.6f, %.6f, %.6f]\n', matlab_out3);

% C++実装
mex_sensor_filter('reset');
cpp_out1 = mex_sensor_filter('gyro', test_input, dt, cutoff);
cpp_out2 = mex_sensor_filter('gyro', test_input, dt, cutoff);
cpp_out3 = mex_sensor_filter('gyro', test_input, dt, cutoff);

fprintf('\nC++ Output (3 iterations):\n');
fprintf('  Out1 = [%.6f, %.6f, %.6f]\n', cpp_out1);
fprintf('  Out2 = [%.6f, %.6f, %.6f]\n', cpp_out2);
fprintf('  Out3 = [%.6f, %.6f, %.6f]\n', cpp_out3);

fprintf('\nDifference:\n');
fprintf('  Diff1 = [%.6f, %.6f, %.6f]\n', matlab_out1 - cpp_out1);
fprintf('  Diff2 = [%.6f, %.6f, %.6f]\n', matlab_out2 - cpp_out2);
fprintf('  Diff3 = [%.6f, %.6f, %.6f]\n', matlab_out3 - cpp_out3);
