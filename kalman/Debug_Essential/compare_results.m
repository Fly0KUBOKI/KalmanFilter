function compare_results()
% COMPARE_RESULTS - 推定結果と真値の比較統計を表示
%
% 使用法:
%   compare_results()
%
% 必要ファイル:
%   - Results/estimation.csv
%   - GenerateData/truth_data.csv
%
% 出力:
%   Position, Velocity, Attitude の RMS/Max エラー

    %% ファイル読み込み
    try
        est_data = readtable('Results/estimation.csv');
        truth_data = readtable('GenerateData/truth_data.csv');
    catch ME
        error('ファイル読み込み失敗: %s', ME.message);
    end
    
    %% データサイズ確認
    n_est = height(est_data);
    n_truth = height(truth_data);
    
    if n_est ~= n_truth
        warning('データサイズ不一致: est=%d, truth=%d', n_est, n_truth);
        n = min(n_est, n_truth);
    else
        n = n_est;
    end
    
    %% Position エラー (XYZ)
    pos_est = [est_data.pos_x(1:n), est_data.pos_y(1:n), est_data.pos_z(1:n)];
    pos_true = [truth_data.x(1:n), truth_data.y(1:n), truth_data.z(1:n)];
    pos_error = pos_est - pos_true;
    
    pos_rms = sqrt(mean(sum(pos_error.^2, 2)));
    [pos_max, pos_max_idx] = max(sqrt(sum(pos_error.^2, 2)));
    
    %% Velocity エラー (XYZ)
    vel_est = [est_data.vel_x(1:n), est_data.vel_y(1:n), est_data.vel_z(1:n)];
    vel_true = [truth_data.vx(1:n), truth_data.vy(1:n), truth_data.vz(1:n)];
    vel_error = vel_est - vel_true;
    
    vel_rms = sqrt(mean(sum(vel_error.^2, 2)));
    [vel_max, vel_max_idx] = max(sqrt(sum(vel_error.^2, 2)));
    
    %% Attitude エラー (Roll/Pitch/Yaw)
    att_est = [est_data.roll(1:n), est_data.pitch(1:n), est_data.yaw(1:n)];
    att_true = [truth_data.roll(1:n), truth_data.pitch(1:n), truth_data.yaw(1:n)];
    
    % 角度差を -180~180 deg に正規化
    att_error = wrapTo180(att_est - att_true);
    
    att_rms = sqrt(mean(att_error.^2, 1));
    [att_max, att_max_idx] = max(abs(att_error), [], 1);
    
    %% 結果表示
    fprintf('\n========== 推定精度サマリー ==========\n\n');
    
    fprintf('【Position】\n');
    fprintf('  RMS Error:  %.4f m\n', pos_rms);
    fprintf('  Max Error:  %.4f m (at step %d, t=%.2f s)\n', ...
        pos_max, pos_max_idx, truth_data.time(pos_max_idx));
    
    fprintf('\n【Velocity】\n');
    fprintf('  RMS Error:  %.4f m/s\n', vel_rms);
    fprintf('  Max Error:  %.4f m/s (at step %d, t=%.2f s)\n', ...
        vel_max, vel_max_idx, truth_data.time(vel_max_idx));
    
    fprintf('\n【Attitude】\n');
    fprintf('  Roll  - RMS: %.4f deg, Max: %.4f deg (at step %d)\n', ...
        att_rms(1), att_max(1), att_max_idx(1));
    fprintf('  Pitch - RMS: %.4f deg, Max: %.4f deg (at step %d)\n', ...
        att_rms(2), att_max(2), att_max_idx(2));
    fprintf('  Yaw   - RMS: %.4f deg, Max: %.4f deg (at step %d)\n', ...
        att_rms(3), att_max(3), att_max_idx(3));
    
    fprintf('\n======================================\n\n');
    
    %% 精度評価
    if pos_rms < 1.0 && vel_rms < 0.5 && all(att_rms < 5.0)
        fprintf('✓ 推定精度: EXCELLENT\n\n');
    elseif pos_rms < 5.0 && vel_rms < 2.0 && all(att_rms < 10.0)
        fprintf('○ 推定精度: GOOD\n\n');
    else
        fprintf('△ 推定精度: NEEDS IMPROVEMENT\n\n');
    end
end
