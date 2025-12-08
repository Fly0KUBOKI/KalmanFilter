% 結果解析スクリプト
fprintf('Loading estimation results...\n');
est = readtable('Results/estimation.csv');
truth = readtable('GenerateData/truth_data.csv');

% 基本統計
fprintf('\n=== Basic Statistics ===\n');
fprintf('Total samples: %d\n', height(est));
fprintf('Time range: %.2f - %.2f seconds\n', est.time(1), est.time(end));

% バイアス推定値の確認
fprintf('\n=== Bias Estimation ===\n');
fprintf('Accel bias (final): [%.6f, %.6f, %.6f] m/s^2\n', ...
    est.ba_x(end), est.ba_y(end), est.ba_z(end));
fprintf('Gyro bias (final): [%.6f, %.6f, %.6f] rad/s\n', ...
    est.bg_x(end), est.bg_y(end), est.bg_z(end));
fprintf('Gyro bias (final, deg/s): [%.6f, %.6f, %.6f]\n', ...
    rad2deg(est.bg_x(end)), rad2deg(est.bg_y(end)), rad2deg(est.bg_z(end)));

% バイアスが変化しているか確認
ba_max = max(abs([est.ba_x; est.ba_y; est.ba_z]));
bg_max = max(abs([est.bg_x; est.bg_y; est.bg_z]));
fprintf('Max accel bias magnitude: %.6f m/s^2\n', ba_max);
fprintf('Max gyro bias magnitude: %.6f rad/s (%.6f deg/s)\n', bg_max, rad2deg(bg_max));

% 誤差計算（初期化期間後）
init_samples = 2000;  % 5秒 @ 400Hz
idx = init_samples+1:height(est);

pos_err = sqrt((est.px(idx) - truth.x(idx)).^2 + ...
               (est.py(idx) - truth.y(idx)).^2 + ...
               (est.pz(idx) - truth.z(idx)).^2);
vel_err = sqrt((est.vx(idx) - truth.vx(idx)).^2 + ...
               (est.vy(idx) - truth.vy(idx)).^2 + ...
               (est.vz(idx) - truth.vz(idx)).^2);

% 姿勢誤差（unwrap処理）
roll_err = rad2deg(wrapToPi(deg2rad(est.roll(idx) - truth.roll(idx))));
pitch_err = rad2deg(wrapToPi(deg2rad(est.pitch(idx) - truth.pitch(idx))));
yaw_err = rad2deg(wrapToPi(deg2rad(est.yaw(idx) - truth.yaw(idx))));

fprintf('\n=== Estimation Errors (after initialization) ===\n');
fprintf('Position RMSE: %.4f m\n', rms(pos_err));
fprintf('Velocity RMSE: %.4f m/s\n', rms(vel_err));
fprintf('Roll RMSE: %.4f deg\n', rms(roll_err));
fprintf('Pitch RMSE: %.4f deg\n', rms(pitch_err));
fprintf('Yaw RMSE: %.4f deg\n', rms(yaw_err));

fprintf('\nPosition Max Error: %.4f m\n', max(pos_err));
fprintf('Velocity Max Error: %.4f m/s\n', max(vel_err));
fprintf('Roll Max Error: %.4f deg\n', max(abs(roll_err)));
fprintf('Pitch Max Error: %.4f deg\n', max(abs(pitch_err)));
fprintf('Yaw Max Error: %.4f deg\n', max(abs(yaw_err)));

% NaN/Infチェック
fprintf('\n=== Divergence Check ===\n');
has_nan = any(isnan(est.px) | isnan(est.py) | isnan(est.pz) | ...
              isnan(est.vx) | isnan(est.vy) | isnan(est.vz) | ...
              isnan(est.roll) | isnan(est.pitch) | isnan(est.yaw));
has_inf = any(isinf(est.px) | isinf(est.py) | isinf(est.pz) | ...
              isinf(est.vx) | isinf(est.vy) | isinf(est.vz));

if has_nan
    fprintf('WARNING: NaN detected in results!\n');
else
    fprintf('No NaN detected\n');
end

if has_inf
    fprintf('WARNING: Inf detected in results!\n');
else
    fprintf('No Inf detected\n');
end

% 判定
fprintf('\n=== Overall Assessment ===\n');
pass = true;
if rms(pos_err) > 5.0
    fprintf('FAIL: Position RMSE too high (%.4f > 5.0 m)\n', rms(pos_err));
    pass = false;
end
if rms(roll_err) > 5.0 || rms(pitch_err) > 5.0
    fprintf('FAIL: Attitude RMSE too high (Roll: %.4f, Pitch: %.4f)\n', ...
        rms(roll_err), rms(pitch_err));
    pass = false;
end
if has_nan || has_inf
    fprintf('FAIL: NaN or Inf detected\n');
    pass = false;
end

if pass
    fprintf('PASS: All checks passed!\n');
else
    fprintf('FAIL: Issues detected\n');
end
