function test_numeric_consistency()
% Phase 1 修正後の数値一貫性テスト
% 目標: This PC と Other PC で Position RMSE の差 < 1e-10 m

fprintf('====================================\n');
fprintf('NUMERIC CONSISTENCY TEST (Phase 1)\n');
fprintf('====================================\n\n');

% テスト用のシード
test_seeds = [1, 42, 123, 999, 2024];

results = struct();

for i = 1:length(test_seeds)
    seed = test_seeds(i);
    fprintf('Testing seed %d... ', seed);
    
    % シミュレーション実行
    tic;
    run_simulation(seed, false);
    elapsed = toc;
    
    % 結果読み込み
    csv_file = sprintf('Results/estimation_%02d.csv', i);
    if ~exist(csv_file, 'file')
        fprintf('FAIL (file not found)\n');
        continue;
    end
    
    data = readmatrix(csv_file);
    
    % Position RMSE 計算
    pos_error = data(:, 2:4);  % [px, py, pz]
    pos_rmse = sqrt(mean(sum(pos_error.^2, 2)));
    
    % Velocity RMSE 計算
    vel_error = data(:, 5:7);  % [vx, vy, vz]
    vel_rmse = sqrt(mean(sum(vel_error.^2, 2)));
    
    % 記録
    results(i).seed = seed;
    results(i).pos_rmse = pos_rmse;
    results(i).vel_rmse = vel_rmse;
    results(i).time = elapsed;
    
    fprintf('PASS (pos=%.10f m, vel=%.10f m/s, time=%.2fs)\n', ...
        pos_rmse, vel_rmse, elapsed);
end

fprintf('\n====================================\n');
fprintf('SUMMARY\n');
fprintf('====================================\n');
fprintf('%-10s %-20s %-20s %-10s\n', 'Seed', 'Position RMSE (m)', 'Velocity RMSE (m/s)', 'Time (s)');
for i = 1:length(results)
    fprintf('%-10d %.15f  %.15f  %.2f\n', ...
        results(i).seed, results(i).pos_rmse, results(i).vel_rmse, results(i).time);
end

% 結果を保存（Other PC との比較用）
save('Results/phase1_numeric_test.mat', 'results');
fprintf('\nResults saved to: Results/phase1_numeric_test.mat\n');
fprintf('Copy this file to Other PC and run compare_numeric_results.m\n');

end
