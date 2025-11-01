function init_errors = initial_state_analysis(obs, truth, estimation, static_samples)
% initial_state_analysis - 初期状態の精度を分析

k = static_samples + 1;

fprintf('--- 初期状態（t=%.4f秒、k=%d）---\n', obs.time(k), k);

p_est = [estimation.px(k); estimation.py(k); estimation.pz(k)];
v_est = [estimation.vx(k); estimation.vy(k); estimation.vz(k)];
euler_est = [estimation.roll(k); estimation.pitch(k); estimation.yaw(k)];

p_true = [truth.x(k); truth.y(k); truth.z(k)];
v_true = [truth.vx(k); truth.vy(k); truth.vz(k)];
euler_true = [truth.roll(k); truth.pitch(k); truth.yaw(k)];

init_errors.p = p_est - p_true;
init_errors.v = v_est - v_true;
init_errors.euler = euler_est - euler_true;

fprintf('\n推定値:\n');
fprintf('  位置: [%.6f, %.6f, %.6f] m\n', p_est(1), p_est(2), p_est(3));
fprintf('  速度: [%.6f, %.6f, %.6f] m/s\n', v_est(1), v_est(2), v_est(3));
fprintf('  姿勢: [%.6f, %.6f, %.6f] deg\n', euler_est(1), euler_est(2), euler_est(3));

fprintf('\n真値:\n');
fprintf('  位置: [%.6f, %.6f, %.6f] m\n', p_true(1), p_true(2), p_true(3));
fprintf('  速度: [%.6f, %.6f, %.6f] m/s\n', v_true(1), v_true(2), v_true(3));
fprintf('  姿勢: [%.6f, %.6f, %.6f] deg\n', euler_true(1), euler_true(2), euler_true(3));

fprintf('\n初期誤差:\n');
fprintf('  位置: [%.6f, %.6f, %.6f] m (ノルム: %.6f)\n', ...
    init_errors.p(1), init_errors.p(2), init_errors.p(3), norm(init_errors.p));
fprintf('  速度: [%.6f, %.6f, %.6f] m/s (ノルム: %.6f)\n', ...
    init_errors.v(1), init_errors.v(2), init_errors.v(3), norm(init_errors.v));
fprintf('  姿勢: [%.6f, %.6f, %.6f] deg (ノルム: %.6f)\n', ...
    init_errors.euler(1), init_errors.euler(2), init_errors.euler(3), norm(init_errors.euler));
end
