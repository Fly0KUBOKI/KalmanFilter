function status = quick_check()
% QUICK_CHECK - 高速な健全性チェック
%
% 使用法:
%   status = quick_check()
%
% 必要ファイル:
%   - Results/estimation.csv
%   - GenerateData/truth_data.csv
%
% 戻り値:
%   status: 0=正常, 1=警告, 2=エラー
%
% チェック項目:
%   1. Position RMS < 5.0 m
%   2. Velocity RMS < 2.0 m/s
%   3. Roll/Pitch RMS < 10.0 deg
%   4. Yaw RMS < 15.0 deg
%   5. データに NaN/Inf が含まれていない

    status = 0; % 初期状態: 正常
    
    fprintf('\n========== Quick Health Check ==========\n');
    
    %% ファイル存在確認
    if ~isfile('Results/estimation.csv') || ~isfile('GenerateData/truth_data.csv')
        fprintf('[FAIL] 必要なファイルが見つかりません\n');
        fprintf('========================================\n\n');
        status = 2;
        return;
    end
    
    %% ファイル読み込み
    try
        est_data = readtable('Results/estimation.csv');
        truth_data = readtable('GenerateData/truth_data.csv');
    catch ME
        fprintf('[FAIL] ファイル読み込みエラー: %s\n', ME.message);
        fprintf('========================================\n\n');
        status = 2;
        return;
    end
    
    %% データサイズ確認
    n = min(height(est_data), height(truth_data));
    if n == 0
        fprintf('[FAIL] データが空です\n');
        fprintf('========================================\n\n');
        status = 2;
        return;
    end
    
    %% NaN/Inf チェック
    has_nan = any(isnan(est_data.px(1:n))) || any(isnan(est_data.vx(1:n))) || ...
              any(isnan(est_data.roll(1:n)));
    has_inf = any(isinf(est_data.px(1:n))) || any(isinf(est_data.vx(1:n))) || ...
              any(isinf(est_data.roll(1:n)));
    
    if has_nan || has_inf
        fprintf('[FAIL] データに NaN/Inf が含まれています\n');
        fprintf('========================================\n\n');
        status = 2;
        return;
    else
        fprintf('[PASS] データ整合性チェック\n');
    end
    
    %% Position エラー
    pos_est = [est_data.px(1:n), est_data.py(1:n), est_data.pz(1:n)];
    pos_true = [truth_data.x(1:n), truth_data.y(1:n), truth_data.z(1:n)];
    pos_error = pos_est - pos_true;
    pos_rms = sqrt(mean(sum(pos_error.^2, 2)));
    
    if pos_rms < 5.0
        fprintf('[PASS] Position RMS: %.4f m\n', pos_rms);
    else
        fprintf('[FAIL] Position RMS: %.4f m (閾値: 5.0 m)\n', pos_rms);
        status = max(status, 2);
    end
    
    %% Velocity エラー
    vel_est = [est_data.vx(1:n), est_data.vy(1:n), est_data.vz(1:n)];
    vel_true = [truth_data.vx(1:n), truth_data.vy(1:n), truth_data.vz(1:n)];
    vel_error = vel_est - vel_true;
    vel_rms = sqrt(mean(sum(vel_error.^2, 2)));
    
    if vel_rms < 2.0
        fprintf('[PASS] Velocity RMS: %.4f m/s\n', vel_rms);
    else
        fprintf('[FAIL] Velocity RMS: %.4f m/s (閾値: 2.0 m/s)\n', vel_rms);
        status = max(status, 2);
    end
    
    %% Attitude エラー
    att_est = [est_data.roll(1:n), est_data.pitch(1:n), est_data.yaw(1:n)];
    att_true = [truth_data.roll(1:n), truth_data.pitch(1:n), truth_data.yaw(1:n)];
    att_error = wrapTo180(att_est - att_true);
    att_rms = sqrt(mean(att_error.^2, 1));
    
    % Roll
    if att_rms(1) < 10.0
        fprintf('[PASS] Roll RMS: %.4f deg\n', att_rms(1));
    else
        fprintf('[WARN] Roll RMS: %.4f deg (閾値: 10.0 deg)\n', att_rms(1));
        status = max(status, 1);
    end
    
    % Pitch
    if att_rms(2) < 10.0
        fprintf('[PASS] Pitch RMS: %.4f deg\n', att_rms(2));
    else
        fprintf('[WARN] Pitch RMS: %.4f deg (閾値: 10.0 deg)\n', att_rms(2));
        status = max(status, 1);
    end
    
    % Yaw
    if att_rms(3) < 15.0
        fprintf('[PASS] Yaw RMS: %.4f deg\n', att_rms(3));
    else
        fprintf('[WARN] Yaw RMS: %.4f deg (閾値: 15.0 deg)\n', att_rms(3));
        status = max(status, 1);
    end
    
    %% 最終判定
    fprintf('========================================\n');
    if status == 0
        fprintf('総合判定: ✓ PASS\n\n');
    elseif status == 1
        fprintf('総合判定: △ WARNING\n\n');
    else
        fprintf('総合判定: × FAIL\n\n');
    end
end
