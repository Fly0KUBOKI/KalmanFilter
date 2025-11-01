function diagnosis_report(bias_stats, sensor_noise, bias_error, init_errors, error_evolution, divergence_info)
% diagnosis_report - 総合診断レポートを生成

fprintf('=== 主な問題点 ===\n\n');

issues = {};
issue_count = 0;

if any(sensor_noise.accel_std > 0.1)
    issue_count = issue_count + 1;
    issues{end+1} = sprintf('問題%d: 加速度計ノイズが大きい (%.4f m/s²)', ...
        issue_count, max(sensor_noise.accel_std));
end

if any(sensor_noise.gyro_std_deg > 1.0)
    issue_count = issue_count + 1;
    issues{end+1} = sprintf('問題%d: ジャイロノイズが大きい (%.4f deg/s)', ...
        issue_count, max(sensor_noise.gyro_std_deg));
end

if abs(bias_error.gyro_mean_error) > 0.1
    issue_count = issue_count + 1;
    issues{end+1} = sprintf('問題%d: ジャイロバイアス補正後も大きな誤差 (%.4f deg/s)', ...
        issue_count, bias_error.gyro_mean_error);
    issues{end+1} = sprintf('  → %.2f秒間で約%.2f度の累積誤差が発生', ...
        error_evolution.time(end), bias_error.accumulated_angle_error);
end

if norm(init_errors.p) > 0.01
    issue_count = issue_count + 1;
    issues{end+1} = sprintf('問題%d: 初期位置誤差が大きい (%.6f m)', ...
        issue_count, norm(init_errors.p));
end

if norm(init_errors.v) > 0.01
    issue_count = issue_count + 1;
    issues{end+1} = sprintf('問題%d: 初期速度誤差が大きい (%.6f m/s)', ...
        issue_count, norm(init_errors.v));
end

if norm(init_errors.euler) > 1.0
    issue_count = issue_count + 1;
    issues{end+1} = sprintf('問題%d: 初期姿勢誤差が大きい (%.4f deg)', ...
        issue_count, norm(init_errors.euler));
end

if divergence_info.pos_diverge_time < inf
    issue_count = issue_count + 1;
    issues{end+1} = sprintf('問題%d: 位置が発散 (t=%.4f秒から)', ...
        issue_count, divergence_info.pos_diverge_time);
end

if divergence_info.vel_diverge_time < inf
    issue_count = issue_count + 1;
    issues{end+1} = sprintf('問題%d: 速度が発散 (t=%.4f秒から)', ...
        issue_count, divergence_info.vel_diverge_time);
end

if divergence_info.att_diverge_time < inf
    issue_count = issue_count + 1;
    issues{end+1} = sprintf('問題%d: 姿勢が発散 (t=%.4f秒から)', ...
        issue_count, divergence_info.att_diverge_time);
end

if isempty(issues)
    fprintf('特に大きな問題は検出されませんでした。\n\n');
else
    for i = 1:length(issues)
        fprintf('%s\n', issues{i});
    end
    fprintf('\n');
end

fprintf('=== 推奨事項 ===\n\n');

if any(sensor_noise.accel_std > 0.1) || any(sensor_noise.gyro_std_deg > 1.0)
    fprintf('・センサーノイズが大きい → ノイズフィルタの適用を検討\n');
    fprintf('  - ローパスフィルタの追加\n');
    fprintf('  - プロセスノイズ行列Qの調整\n');
end

if abs(bias_error.gyro_mean_error) > 0.1
    fprintf('・ジャイロバイアスの補正精度が低い\n');
    fprintf('  - 静止期間を延長してより正確なバイアス推定\n');
    fprintf('  - バイアスをオンラインで推定・更新\n');
    fprintf('  - ランダムウォークモデルの導入\n');
end

if norm(init_errors.v) > 0.01 || norm(init_errors.euler) > 1.0
    fprintf('・初期状態の精度が低い\n');
    fprintf('  - 初期共分散行列Pの調整\n');
    fprintf('  - より正確な初期姿勢推定手法の採用\n');
end

if divergence_info.att_diverge_time < divergence_info.pos_diverge_time && ...
   divergence_info.att_diverge_time < divergence_info.vel_diverge_time
    fprintf('・姿勢が最初に発散 → 姿勢推定の改善が最優先\n');
    fprintf('  - 観測更新の頻度を増やす\n');
    fprintf('  - 観測ノイズ行列Rの調整\n');
elseif divergence_info.vel_diverge_time < divergence_info.pos_diverge_time
    fprintf('・速度が位置より先に発散 → 速度推定の改善が重要\n');
    fprintf('  - GPS速度の観測を追加\n');
    fprintf('  - 速度に関するプロセスノイズの調整\n');
end

fprintf('\n');
end
