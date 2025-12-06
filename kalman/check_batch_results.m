% batch結果の詳細確認
load('Results/batch_results.mat');

fprintf('=== Detailed Results ===\n');
for i = 1:length(results_summary)
    fprintf('\nRun %d:\n', i);
    if isfield(results_summary(i), 'error')
        fprintf('  ERROR: %s\n', results_summary(i).error);
    else
        fprintf('  Position RMSE: %.4f m\n', results_summary(i).pos_rmse);
        fprintf('  Roll RMSE: %.4f deg\n', results_summary(i).roll_rmse);
        fprintf('  Pitch RMSE: %.4f deg\n', results_summary(i).pitch_rmse);
        fprintf('  Yaw RMSE: %.4f deg\n', results_summary(i).yaw_rmse);
        fprintf('  Gyro bias (final): [%.4f, %.4f, %.4f] deg/s\n', rad2deg(results_summary(i).bg_final));
        fprintf('  Has NaN: %d\n', results_summary(i).has_nan);
        fprintf('  Has Inf: %d\n', results_summary(i).has_inf);
        fprintf('  Max Innovation: %.4f\n', results_summary(i).max_innov);
        fprintf('  Max Mahalanobis: %.4f\n', results_summary(i).max_maha);
        
        % 判定
        if results_summary(i).has_nan || results_summary(i).has_inf
            fprintf('  Status: FAILED (NaN/Inf)\n');
        elseif results_summary(i).pos_rmse > 5.0 || results_summary(i).roll_rmse > 5.0 || results_summary(i).pitch_rmse > 5.0
            fprintf('  Status: FAILED (Error too high)\n');
        else
            fprintf('  Status: PASS\n');
        end
    end
end
