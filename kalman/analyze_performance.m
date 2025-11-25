function analyze_performance()
    % パフォーマンス解析スクリプト
    
    clc;
    close all;
    
    % プロジェクトルート
    proj_root = fileparts(mfilename('fullpath'));
    addpath(genpath(proj_root));
    
    % 設定ファイルのパス
    config_file = fullfile(proj_root, 'GenerateData', 'config_params.m');
    
    % ログ記録開始
    diary('analysis_log.txt');
    
    % 1. ノイズなしシミュレーション
    fprintf('=== ノイズなしシミュレーション開始 ===\n');
    set_noise_enable(config_file, false);
    try
        run_simulation();
        [t_clean, err_clean, res_clean, truth_clean] = load_and_calc_error(proj_root);
        plot_errors(t_clean, err_clean, res_clean, truth_clean, 'Noise-Free');
    catch ME
        fprintf('Error in noise-free simulation: %s\n', ME.message);
    end
    
    % 2. ノイズありシミュレーション
    fprintf('\n=== ノイズありシミュレーション開始 ===\n');
    set_noise_enable(config_file, true);
    try
        run_simulation();
        [t_noisy, err_noisy, res_noisy, truth_noisy] = load_and_calc_error(proj_root);
        plot_errors(t_noisy, err_noisy, res_noisy, truth_noisy, 'With Noise');
    catch ME
        fprintf('Error in noisy simulation: %s\n', ME.message);
    end
    
    % 設定を元に戻す（ノイズあり）
    set_noise_enable(config_file, true);
end

function set_noise_enable(config_file, enable)
    % config_params.m のノイズ設定を書き換える
    % enable: true or false
    
    content = fileread(config_file);
    
    if enable
        val_str = 'true';
        baro_val = 'false'; % Baro is default false
    else
        val_str = 'false';
        baro_val = 'false';
    end
    
    % 正規表現で置換
    % params.noise.enable.accel = ...;
    content = regexprep(content, 'params.noise.enable.accel = \w+;', sprintf('params.noise.enable.accel = %s;', val_str));
    content = regexprep(content, 'params.noise.enable.gyro = \w+;', sprintf('params.noise.enable.gyro = %s;', val_str));
    content = regexprep(content, 'params.noise.enable.mag = \w+;', sprintf('params.noise.enable.mag = %s;', val_str));
    content = regexprep(content, 'params.noise.enable.baro = \w+;', sprintf('params.noise.enable.baro = %s;', baro_val));
    content = regexprep(content, 'params.noise.enable.gps = \w+;', sprintf('params.noise.enable.gps = %s;', val_str));
    content = regexprep(content, 'params.noise.enable.outlier = \w+;', sprintf('params.noise.enable.outlier = %s;', val_str));
    content = regexprep(content, 'params.noise.enable.pink = \w+;', sprintf('params.noise.enable.pink = %s;', val_str));
    content = regexprep(content, 'params.noise.enable.allan = \w+;', sprintf('params.noise.enable.allan = %s;', val_str));
    
    fid = fopen(config_file, 'w');
    fwrite(fid, content);
    fclose(fid);
    
    % キャッシュクリア
    clear config_params;
    rehash;
end

function [t, err, res, truth] = load_and_calc_error(proj_root)
    % 結果と真値を読み込んで誤差を計算
    
    res_file = fullfile(proj_root, 'Results', 'estimation.csv');
    truth_file = fullfile(proj_root, 'GenerateData', 'truth_data.csv');
    
    res_table = readtable(res_file);
    truth_table = readtable(truth_file);
    
    % 時間合わせ（必要ならインターポレーション）
    % ここではサンプル数が同じと仮定
    n = min(height(res_table), height(truth_table));
    res_table = res_table(1:n, :);
    truth_table = truth_table(1:n, :);
    
    t = res_table.time;
    
    % 構造体に変換
    res.p = [res_table.px, res_table.py, res_table.pz];
    res.v = [res_table.vx, res_table.vy, res_table.vz];
    res.euler = [res_table.roll, res_table.pitch, res_table.yaw];
    
    truth.p = [truth_table.p_x, truth_table.p_y, truth_table.p_z];
    truth.v = [truth_table.v_x, truth_table.v_y, truth_table.v_z];
    truth.euler = [truth_table.roll, truth_table.pitch, truth_table.yaw];
    
    % 誤差計算
    err.p = res.p - truth.p;
    err.v = res.v - truth.v;
    
    % 角度誤差（ラップアラウンド考慮）
    err.euler = res.euler - truth.euler;
    err.euler = rad2deg(angdiff(deg2rad(res.euler), deg2rad(truth.euler)));
    
    % RMSE計算
    rmse_p = sqrt(mean(sum(err.p.^2, 2)));
    rmse_v = sqrt(mean(sum(err.v.^2, 2)));
    rmse_att = sqrt(mean(sum(err.euler.^2, 2)));
    
    fprintf('RMSE Position: %.3f m\n', rmse_p);
    fprintf('RMSE Velocity: %.3f m/s\n', rmse_v);
    fprintf('RMSE Attitude: %.3f deg\n', rmse_att);
end

function plot_errors(t, err, res, truth, title_prefix)
    figure('Name', [title_prefix ' - Position'], 'NumberTitle', 'off');
    subplot(3,1,1); plot(t, res.p(:,1), 'b', t, truth.p(:,1), 'r--'); title('Pos X'); grid on; legend('Est', 'Truth');
    subplot(3,1,2); plot(t, res.p(:,2), 'b', t, truth.p(:,2), 'r--'); title('Pos Y'); grid on;
    subplot(3,1,3); plot(t, res.p(:,3), 'b', t, truth.p(:,3), 'r--'); title('Pos Z'); grid on;
    
    figure('Name', [title_prefix ' - Attitude'], 'NumberTitle', 'off');
    subplot(3,1,1); plot(t, res.euler(:,1), 'b', t, truth.euler(:,1), 'r--'); title('Roll'); grid on; legend('Est', 'Truth');
    subplot(3,1,2); plot(t, res.euler(:,2), 'b', t, truth.euler(:,2), 'r--'); title('Pitch'); grid on;
    subplot(3,1,3); plot(t, res.euler(:,3), 'b', t, truth.euler(:,3), 'r--'); title('Yaw'); grid on;
    
    figure('Name', [title_prefix ' - Errors'], 'NumberTitle', 'off');
    subplot(3,1,1); plot(t, err.p); title('Position Error'); legend('X','Y','Z'); grid on;
    subplot(3,1,2); plot(t, err.v); title('Velocity Error'); legend('X','Y','Z'); grid on;
    subplot(3,1,3); plot(t, err.euler); title('Attitude Error'); legend('R','P','Y'); grid on;
    
    drawnow;
end
