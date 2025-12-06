% 5回のシミュレーション実行スクリプト
results_summary = struct();

for run_id = 1:5
    fprintf('\n========================================\n');
    fprintf('=== Run %d/5 ===\n', run_id);
    fprintf('========================================\n');
    
    try
        % シミュレーション実行
        run_simulation(run_id, false);
        
        % 結果読み込み
        est = readtable('Results/estimation.csv');
        truth = readtable('GenerateData/truth_data.csv');
        
        % 初期化期間後のインデックス
        init_samples = 2000;
        idx = init_samples+1:height(est);
        
        % 誤差計算
        pos_err = sqrt((est.px(idx) - truth.x(idx)).^2 + ...
                       (est.py(idx) - truth.y(idx)).^2 + ...
                       (est.pz(idx) - truth.z(idx)).^2);
        vel_err = sqrt((est.vx(idx) - truth.vx(idx)).^2 + ...
                       (est.vy(idx) - truth.vy(idx)).^2 + ...
                       (est.vz(idx) - truth.vz(idx)).^2);
        
        roll_err = rad2deg(wrapToPi(deg2rad(est.roll(idx) - truth.roll(idx))));
        pitch_err = rad2deg(wrapToPi(deg2rad(est.pitch(idx) - truth.pitch(idx))));
        yaw_err = rad2deg(wrapToPi(deg2rad(est.yaw(idx) - truth.yaw(idx))));
        
        % 結果保存
        results_summary(run_id).pos_rmse = rms(pos_err);
        results_summary(run_id).vel_rmse = rms(vel_err);
        results_summary(run_id).roll_rmse = rms(roll_err);
        results_summary(run_id).pitch_rmse = rms(pitch_err);
        results_summary(run_id).yaw_rmse = rms(yaw_err);
        results_summary(run_id).ba_final = [est.ba_x(end), est.ba_y(end), est.ba_z(end)];
        results_summary(run_id).bg_final = [est.bg_x(end), est.bg_y(end), est.bg_z(end)];
        results_summary(run_id).has_nan = any(isnan(est.px) | isnan(est.py) | isnan(est.pz));
        results_summary(run_id).has_inf = any(isinf(est.px) | isinf(est.py) | isinf(est.pz));
        results_summary(run_id).max_innov = max(est.innov_norm);
        results_summary(run_id).max_maha = max(est.maha_dist);
        
        fprintf('Run %d Summary:\n', run_id);
        fprintf('  Position RMSE: %.4f m\n', results_summary(run_id).pos_rmse);
        fprintf('  Roll/Pitch/Yaw RMSE: %.4f / %.4f / %.4f deg\n', ...
            results_summary(run_id).roll_rmse, results_summary(run_id).pitch_rmse, results_summary(run_id).yaw_rmse);
        fprintf('  Gyro bias (final): [%.4f, %.4f, %.4f] deg/s\n', ...
            rad2deg(results_summary(run_id).bg_final));
        fprintf('  Status: SUCCESS\n');
        
    catch e
        fprintf('Run %d FAILED: %s\n', run_id, e.message);
        results_summary(run_id).error = e.message;
        results_summary(run_id).pos_rmse = NaN;
    end
end

% 総合結果
fprintf('\n========================================\n');
fprintf('=== Summary of All Runs ===\n');
fprintf('========================================\n');

pos_rmse_all = [results_summary.pos_rmse];
roll_rmse_all = [results_summary.roll_rmse];
pitch_rmse_all = [results_summary.pitch_rmse];
yaw_rmse_all = [results_summary.yaw_rmse];

fprintf('Position RMSE: Mean=%.4f, Std=%.4f, Max=%.4f m\n', ...
    mean(pos_rmse_all), std(pos_rmse_all), max(pos_rmse_all));
fprintf('Roll RMSE: Mean=%.4f, Std=%.4f, Max=%.4f deg\n', ...
    mean(roll_rmse_all), std(roll_rmse_all), max(roll_rmse_all));
fprintf('Pitch RMSE: Mean=%.4f, Std=%.4f, Max=%.4f deg\n', ...
    mean(pitch_rmse_all), std(pitch_rmse_all), max(pitch_rmse_all));
fprintf('Yaw RMSE: Mean=%.4f, Std=%.4f, Max=%.4f deg\n', ...
    mean(yaw_rmse_all), std(yaw_rmse_all), max(yaw_rmse_all));

% 判定
all_pass = true;
for i = 1:5
    if isfield(results_summary(i), 'error') || results_summary(i).has_nan || results_summary(i).has_inf
        all_pass = false;
        fprintf('Run %d: FAILED\n', i);
    elseif results_summary(i).pos_rmse > 5.0 || results_summary(i).roll_rmse > 5.0 || results_summary(i).pitch_rmse > 5.0
        all_pass = false;
        fprintf('Run %d: Error too high\n', i);
    else
        fprintf('Run %d: PASS\n', i);
    end
end

if all_pass
    fprintf('\n=== OVERALL: ALL RUNS PASSED ===\n');
else
    fprintf('\n=== OVERALL: SOME RUNS FAILED ===\n');
end

% 結果を保存
save('Results/batch_results.mat', 'results_summary');
fprintf('Results saved to Results/batch_results.mat\n');
