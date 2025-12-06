function analyze_divergence()
    % 推定結果の詳細解析プログラム
    % 姿勢・速度・位置の発散を詳細に分析
    
    fprintf('=== 推定結果の詳細解析 ===\n\n');
    
    % データ読み込み
    est = readtable('Results/estimation.csv');
    truth = readtable('GenerateData/truth_data.csv');
    sensor = readtable('GenerateData/sensor_data.csv');
    
    % 誤差計算
    err_p = sqrt((est.px - truth.x).^2 + (est.py - truth.y).^2 + (est.pz - truth.z).^2);
    err_v = sqrt((est.vx - truth.vx).^2 + (est.vy - truth.vy).^2 + (est.vz - truth.vz).^2);
    
    % 姿勢誤差（角度ラップ考慮）
    err_roll = wrapToPi(est.roll - truth.roll);
    err_pitch = wrapToPi(est.pitch - truth.pitch);
    err_yaw = wrapToPi(est.yaw - truth.yaw);
    err_att = sqrt(err_roll.^2 + err_pitch.^2 + err_yaw.^2);
    
    % 統計サマリー
    fprintf('【統計サマリー】\n');
    fprintf('位置誤差: 平均=%.2fm, 最大=%.2fm, 標準偏差=%.2fm\n', ...
        mean(err_p), max(err_p), std(err_p));
    fprintf('速度誤差: 平均=%.3fm/s, 最大=%.3fm/s, 標準偏差=%.3fm/s\n', ...
        mean(err_v), max(err_v), std(err_v));
    fprintf('姿勢誤差: 平均=%.2fdeg, 最大=%.2fdeg, 標準偏差=%.2fdeg\n\n', ...
        rad2deg(mean(err_att)), rad2deg(max(err_att)), rad2deg(std(err_att)));
    
    % 発散検出
    [is_diverged, div_info] = detect_divergence(est, truth, err_p, err_v, err_att);
    
    if is_diverged
        fprintf('【警告】推定が発散しています！\n');
        fprintf('発散開始時刻: %.2fs\n', div_info.start_time);
        fprintf('発散タイプ: %s\n', div_info.type);
        fprintf('最大姿勢誤差: %.1fdeg (t=%.2fs)\n', ...
            rad2deg(div_info.max_att_error), div_info.max_att_time);
        fprintf('最大速度誤差: %.2fm/s (t=%.2fs)\n', ...
            div_info.max_vel_error, div_info.max_vel_time);
        fprintf('\n');
        
        % 発散区間の詳細分析
        analyze_divergence_period(est, truth, sensor, div_info);
    else
        fprintf('推定は安定しています。\n\n');
    end
    
    % 時系列プロット
    plot_time_series(est, truth, err_p, err_v, err_att);
    
    % 姿勢発散の詳細分析
    analyze_attitude_divergence(est, truth);
    
    % 発散パラメータの解析
    analyze_divergence_parameters(est);
    
    % 中盤の発散詳細解析
    analyze_midpoint_divergence(est, truth, sensor, err_p, err_v, err_att);
    
    fprintf('\n解析完了。\n');
end

function [is_diverged, div_info] = detect_divergence(est, truth, err_p, err_v, err_att)
    % 発散検出
    
    div_info = struct();
    
    % 発散判定閾値
    threshold_att = deg2rad(30);  % 姿勢誤差30度以上
    threshold_vel = 1.0;          % 速度誤差1.0m/s以上
    threshold_pos = 5.0;          % 位置誤差5.0m以上
    
    % 発散インデックス検出
    div_idx_att = find(err_att > threshold_att);
    div_idx_vel = find(err_v > threshold_vel);
    div_idx_pos = find(err_p > threshold_pos);
    
    is_diverged = ~isempty(div_idx_att) || ~isempty(div_idx_vel) || ~isempty(div_idx_pos);
    
    if is_diverged
        % 発散開始時刻
        if ~isempty(div_idx_att)
            div_info.start_time = est.time(div_idx_att(1));
            div_info.type = '姿勢発散';
        elseif ~isempty(div_idx_vel)
            div_info.start_time = est.time(div_idx_vel(1));
            div_info.type = '速度発散';
        else
            div_info.start_time = est.time(div_idx_pos(1));
            div_info.type = '位置発散';
        end
        
        % 最大誤差
        [div_info.max_att_error, max_att_idx] = max(err_att);
        div_info.max_att_time = est.time(max_att_idx);
        
        [div_info.max_vel_error, max_vel_idx] = max(err_v);
        div_info.max_vel_time = est.time(max_vel_idx);
        
        [div_info.max_pos_error, max_pos_idx] = max(err_p);
        div_info.max_pos_time = est.time(max_pos_idx);
    else
        div_info.start_time = NaN;
        div_info.type = 'なし';
        div_info.max_att_error = max(err_att);
        div_info.max_att_time = est.time(end);
        div_info.max_vel_error = max(err_v);
        div_info.max_vel_time = est.time(end);
        div_info.max_pos_error = max(err_p);
        div_info.max_pos_time = est.time(end);
    end
end

function analyze_divergence_period(est, truth, sensor, div_info)
    % 発散区間の詳細分析
    
    fprintf('【発散区間の詳細分析】\n');
    
    % 発散開始前後10秒の区間を分析
    t_start = max(0, div_info.start_time - 5);
    t_end = min(est.time(end), div_info.start_time + 15);
    
    idx_range = find(est.time >= t_start & est.time <= t_end);
    
    fprintf('分析区間: %.1fs ~ %.1fs\n\n', t_start, t_end);
    fprintf('時刻[s] | Roll誤差[deg] | Pitch誤差[deg] | Yaw誤差[deg] | 速度誤差[m/s]\n');
    fprintf('--------|---------------|----------------|--------------|-------------\n');
    
    for i = 1:min(length(idx_range), 30)  % 最大30サンプル表示
        k = idx_range(i);
        if mod(i, 3) == 1  % 3サンプルごとに表示
            err_roll = rad2deg(wrapToPi(est.roll(k) - truth.roll(k)));
            err_pitch = rad2deg(wrapToPi(est.pitch(k) - truth.pitch(k)));
            err_yaw = rad2deg(wrapToPi(est.yaw(k) - truth.yaw(k)));
            err_v = sqrt((est.vx(k) - truth.vx(k))^2 + ...
                        (est.vy(k) - truth.vy(k))^2 + ...
                        (est.vz(k) - truth.vz(k))^2);
            
            fprintf('%7.2f | %13.1f | %14.1f | %12.1f | %11.3f\n', ...
                est.time(k), err_roll, err_pitch, err_yaw, err_v);
        end
    end
    fprintf('\n');
    
    % 加速度計更新の影響分析
    if isfield(est, 'P_vx')
        fprintf('【共分散の推移】\n');
        fprintf('時刻[s] | P_vx | P_vy | P_vz | P_att_max\n');
        fprintf('--------|------|------|------|-----------\n');
        for i = 1:min(length(idx_range), 20)
            k = idx_range(i);
            if mod(i, 5) == 1
                P_att_cols = 4:6;  % 姿勢の共分散は4~6列目（仮定）
                if size(est, 2) >= 6
                    fprintf('%7.2f | %.2e | %.2e | %.2e | %.2e\n', ...
                        est.time(k), est.P_vx(k), est.P_vy(k), est.P_vz(k), ...
                        max([est.P_vx(k), est.P_vy(k), est.P_vz(k)]));
                end
            end
        end
        fprintf('\n');
    end
end

function analyze_attitude_divergence(est, truth)
    % 姿勢発散の詳細分析
    
    fprintf('【姿勢発散の詳細分析】\n');
    
    % 姿勢の時系列変化を確認
    err_roll = rad2deg(wrapToPi(est.roll - truth.roll));
    err_pitch = rad2deg(wrapToPi(est.pitch - truth.pitch));
    err_yaw = rad2deg(wrapToPi(est.yaw - truth.yaw));
    
    % 大きな姿勢誤差が発生している時刻を検出
    large_err_idx = find(abs(err_roll) > 20 | abs(err_pitch) > 20);
    
    if ~isempty(large_err_idx)
        fprintf('大きな姿勢誤差が検出されました（>20度）\n');
        fprintf('発生サンプル数: %d / %d (%.1f%%)\n', ...
            length(large_err_idx), length(est.time), ...
            100 * length(large_err_idx) / length(est.time));
        fprintf('初回発生時刻: %.2fs\n', est.time(large_err_idx(1)));
        fprintf('\n');
        
        % 姿勢ジャンプの検出（連続サンプル間の大きな変化）
        d_roll = diff(est.roll);
        d_pitch = diff(est.pitch);
        
        large_jump_idx = find(abs(d_roll) > deg2rad(10) | abs(d_pitch) > deg2rad(10));
        
        if ~isempty(large_jump_idx)
            fprintf('急激な姿勢変化（ジャンプ）が検出されました\n');
            fprintf('発生回数: %d回\n', length(large_jump_idx));
            fprintf('最初の5回:\n');
            for i = 1:min(5, length(large_jump_idx))
                k = large_jump_idx(i);
                fprintf('  t=%.2fs: ΔRoll=%.1fdeg, ΔPitch=%.1fdeg\n', ...
                    est.time(k), rad2deg(d_roll(k)), rad2deg(d_pitch(k)));
            end
            fprintf('\n');
        end
    else
        fprintf('大きな姿勢誤差は検出されませんでした。\n\n');
    end
end

function plot_time_series(est, truth, err_p, err_v, err_att)
    % 時系列プロット
    
    figure('Name', '詳細解析: 誤差時系列', 'Position', [100, 100, 1200, 800]);
    
    % 位置誤差
    subplot(3, 1, 1);
    plot(est.time, err_p, 'b-', 'LineWidth', 1.5);
    grid on;
    xlabel('時刻 [s]');
    ylabel('位置誤差 [m]');
    title('位置推定誤差の時系列');
    ylim([0, max(err_p) * 1.1]);
    
    % 速度誤差
    subplot(3, 1, 2);
    plot(est.time, err_v, 'r-', 'LineWidth', 1.5);
    grid on;
    xlabel('時刻 [s]');
    ylabel('速度誤差 [m/s]');
    title('速度推定誤差の時系列');
    ylim([0, max(err_v) * 1.1]);
    
    % 姿勢誤差
    subplot(3, 1, 3);
    plot(est.time, rad2deg(err_att), 'g-', 'LineWidth', 1.5);
    hold on;
    plot(est.time, rad2deg(abs(wrapToPi(est.roll - truth.roll))), '--', 'Color', [0.5, 0, 0]);
    plot(est.time, rad2deg(abs(wrapToPi(est.pitch - truth.pitch))), '--', 'Color', [0, 0.5, 0]);
    plot(est.time, rad2deg(abs(wrapToPi(est.yaw - truth.yaw))), '--', 'Color', [0, 0, 0.5]);
    hold off;
    grid on;
    xlabel('時刻 [s]');
    ylabel('姿勢誤差 [deg]');
    title('姿勢推定誤差の時系列');
    legend('総合', 'Roll', 'Pitch', 'Yaw', 'Location', 'best');
    ylim([0, min(rad2deg(max(err_att)) * 1.1, 180)]);
end

function analyze_divergence_parameters(est)
    % 発散パラメータの時系列解析
    
    fprintf('【発散パラメータの解析】\n');
    
    % パラメータの統計
    fprintf('イノベーションノルム: 平均=%.3f, 最大=%.3f, 標準偏差=%.3f\n', ...
        mean(est.innov_norm), max(est.innov_norm), std(est.innov_norm));
    fprintf('マハラノビス距離: 平均=%.3f, 最大=%.3f, 標準偏差=%.3f\n', ...
        mean(est.maha_dist), max(est.maha_dist), std(est.maha_dist));
    fprintf('ゲインノルム: 平均=%.3f, 最大=%.3f, 標準偏差=%.3f\n', ...
        mean(est.gain_norm), max(est.gain_norm), std(est.gain_norm));
    fprintf('クォータニオンノルム: 平均=%.6f, 最小=%.6f, 最大=%.6f\n', ...
        mean(est.quat_norm), min(est.quat_norm), max(est.quat_norm));
    fprintf('姿勢変化率: 平均=%.6f, 最大=%.6f, 標準偏差=%.6f\n\n', ...
        mean(est.att_change_rate), max(est.att_change_rate), std(est.att_change_rate));
    
    % 異常値の検出
    high_innov_idx = find(est.innov_norm > 0.5);
    high_maha_idx = find(est.maha_dist > 10.0);
    high_gain_idx = find(est.gain_norm > 2.0);
    quat_error_idx = find(abs(est.quat_norm - 1.0) > 0.01);
    
    fprintf('異常パラメータの検出:\n');
    fprintf('  高イノベーション(>0.5): %d回 (%.1f%%)\n', ...
        length(high_innov_idx), 100*length(high_innov_idx)/length(est.time));
    fprintf('  高マハラノビス(>10.0): %d回 (%.1f%%)\n', ...
        length(high_maha_idx), 100*length(high_maha_idx)/length(est.time));
    fprintf('  高ゲイン(>2.0): %d回 (%.1f%%)\n', ...
        length(high_gain_idx), 100*length(high_gain_idx)/length(est.time));
    fprintf('  クォータニオン誤差(>0.01): %d回 (%.1f%%)\n\n', ...
        length(quat_error_idx), 100*length(quat_error_idx)/length(est.time));
    
    if ~isempty(high_innov_idx)
        fprintf('高イノベーション発生時刻（最初の10回）:\n');
        for i = 1:min(10, length(high_innov_idx))
            k = high_innov_idx(i);
            fprintf('  t=%.2fs: innov=%.3f, maha=%.3f, gain=%.3f\n', ...
                est.time(k), est.innov_norm(k), est.maha_dist(k), est.gain_norm(k));
        end
        fprintf('\n');
    end
    
    % 発散パラメータのプロット
    figure('Name', '発散パラメータ時系列', 'Position', [150, 150, 1200, 900]);
    
    subplot(4, 1, 1);
    plot(est.time, est.innov_norm, 'b-');
    hold on;
    plot(est.time, ones(size(est.time))*0.5, 'r--', 'LineWidth', 1.5);
    hold off;
    grid on;
    ylabel('イノベーションノルム');
    title('発散パラメータの時系列');
    legend('innov\_norm', '閾値(0.5)');
    
    subplot(4, 1, 2);
    plot(est.time, est.maha_dist, 'g-');
    hold on;
    plot(est.time, ones(size(est.time))*10.0, 'r--', 'LineWidth', 1.5);
    hold off;
    grid on;
    ylabel('マハラノビス距離');
    legend('maha\_dist', '閾値(10.0)');
    
    subplot(4, 1, 3);
    plot(est.time, est.gain_norm, 'm-');
    hold on;
    plot(est.time, ones(size(est.time))*2.0, 'r--', 'LineWidth', 1.5);
    hold off;
    grid on;
    ylabel('ゲインノルム');
    legend('gain\_norm', '閾値(2.0)');
    
    subplot(4, 1, 4);
    plot(est.time, abs(est.quat_norm - 1.0), 'c-');
    hold on;
    plot(est.time, ones(size(est.time))*0.01, 'r--', 'LineWidth', 1.5);
    hold off;
    grid on;
    xlabel('時刻 [s]');
    ylabel('|quat\_norm - 1.0|');
    legend('正規化誤差', '閾値(0.01)');
end

function analyze_midpoint_divergence(est, truth, sensor, err_p, err_v, err_att)
    % 中盤の発散詳細解析
    
    fprintf('【中盤の発散詳細解析】\n');
    
    % 時系列を10秒ごとの区間に分割
    total_time = est.time(end);
    n_segments = floor(total_time / 10);
    
    fprintf('全体時間: %.1fs を %d 区間に分割して解析\n\n', total_time, n_segments);
    fprintf('区間 | 時刻範囲[s] | 位置誤差[m] | 速度誤差[m/s] | 姿勢誤差[deg] | 最大innov | 最大maha | 最大gain\n');
    fprintf('-----|-------------|-------------|---------------|---------------|-----------|----------|----------\n');
    
    max_err_segments = [];
    
    for seg = 1:n_segments
        t_start = (seg - 1) * 10;
        t_end = seg * 10;
        idx = find(est.time >= t_start & est.time < t_end);
        
        if ~isempty(idx)
            avg_err_p = mean(err_p(idx));
            max_err_p = max(err_p(idx));
            avg_err_v = mean(err_v(idx));
            max_err_v = max(err_v(idx));
            avg_err_att = mean(err_att(idx));
            max_err_att = max(err_att(idx));
            max_innov = max(est.innov_norm(idx));
            max_maha = max(est.maha_dist(idx));
            max_gain = max(est.gain_norm(idx));
            
            fprintf('%4d | %4.0f - %4.0f | %5.2f(%5.2f) | %6.3f(%6.3f) | %7.2f(%7.2f) | %9.3f | %8.3f | %8.3f\n', ...
                seg, t_start, t_end, avg_err_p, max_err_p, avg_err_v, max_err_v, ...
                rad2deg(avg_err_att), rad2deg(max_err_att), max_innov, max_maha, max_gain);
            
            % 誤差が大きい区間を記録
            if max_err_p > 5.0 || max_err_v > 1.0 || rad2deg(max_err_att) > 30
                max_err_segments = [max_err_segments; seg, t_start, t_end, max_err_p, max_err_v, rad2deg(max_err_att)];
            end
        end
    end
    
    fprintf('\n');
    
    % 誤差が大きい区間の詳細分析
    if ~isempty(max_err_segments)
        fprintf('【高誤差区間の詳細分析】\n');
        fprintf('以下の%d区間で大きな誤差が検出されました:\n\n', size(max_err_segments, 1));
        
        for i = 1:min(5, size(max_err_segments, 1))
            seg = max_err_segments(i, 1);
            t_start = max_err_segments(i, 2);
            t_end = max_err_segments(i, 3);
            
            fprintf('--- 区間%d: %.0fs-%.0fs ---\n', seg, t_start, t_end);
            fprintf('最大位置誤差: %.2fm, 最大速度誤差: %.3fm/s, 最大姿勢誤差: %.1fdeg\n', ...
                max_err_segments(i, 4), max_err_segments(i, 5), max_err_segments(i, 6));
            
            % この区間の詳細データ
            idx = find(est.time >= t_start & est.time < t_end);
            
            % 最大誤差発生時刻を特定
            [~, max_p_idx] = max(err_p(idx));
            [~, max_v_idx] = max(err_v(idx));
            [~, max_att_idx] = max(err_att(idx));
            
            k_max_p = idx(max_p_idx);
            k_max_v = idx(max_v_idx);
            k_max_att = idx(max_att_idx);
            
            fprintf('  位置誤差ピーク: t=%.2fs (px=%.2f, py=%.2f, pz=%.2f)\n', ...
                est.time(k_max_p), est.px(k_max_p)-truth.x(k_max_p), ...
                est.py(k_max_p)-truth.y(k_max_p), est.pz(k_max_p)-truth.z(k_max_p));
            fprintf('  速度誤差ピーク: t=%.2fs (vx=%.2f, vy=%.2f, vz=%.2f)\n', ...
                est.time(k_max_v), est.vx(k_max_v)-truth.vx(k_max_v), ...
                est.vy(k_max_v)-truth.vy(k_max_v), est.vz(k_max_v)-truth.vz(k_max_v));
            fprintf('  姿勢誤差ピーク: t=%.2fs (roll=%.1f, pitch=%.1f, yaw=%.1f)\n', ...
                est.time(k_max_att), rad2deg(wrapToPi(est.roll(k_max_att)-truth.roll(k_max_att))), ...
                rad2deg(wrapToPi(est.pitch(k_max_att)-truth.pitch(k_max_att))), ...
                rad2deg(wrapToPi(est.yaw(k_max_att)-truth.yaw(k_max_att))));
            
            % 発散パラメータ
            fprintf('  発散パラメータ:\n');
            fprintf('    イノベーション: 平均=%.3f, 最大=%.3f\n', ...
                mean(est.innov_norm(idx)), max(est.innov_norm(idx)));
            fprintf('    マハラノビス: 平均=%.3f, 最大=%.3f\n', ...
                mean(est.maha_dist(idx)), max(est.maha_dist(idx)));
            fprintf('    ゲインノルム: 平均=%.3f, 最大=%.3f\n', ...
                mean(est.gain_norm(idx)), max(est.gain_norm(idx)));
            fprintf('    クォータニオン: 最小=%.6f, 最大=%.6f\n', ...
                min(est.quat_norm(idx)), max(est.quat_norm(idx)));
            
            % 急激な変化の検出
            if length(idx) > 1
                d_roll = diff(est.roll(idx));
                d_pitch = diff(est.pitch(idx));
                d_vx = diff(est.vx(idx));
                d_vy = diff(est.vy(idx));
                
                large_jump = find(abs(d_roll) > deg2rad(5) | abs(d_pitch) > deg2rad(5));
                if ~isempty(large_jump)
                    fprintf('    急激な姿勢変化: %d回検出\n', length(large_jump));
                end
                
                large_vel_jump = find(abs(d_vx) > 0.1 | abs(d_vy) > 0.1);
                if ~isempty(large_vel_jump)
                    fprintf('    急激な速度変化: %d回検出\n', length(large_vel_jump));
                end
            end
            
            fprintf('\n');
        end
    else
        fprintf('すべての区間で誤差は許容範囲内です。\n\n');
    end
end
