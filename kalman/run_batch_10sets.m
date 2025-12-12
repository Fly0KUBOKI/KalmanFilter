% run_batch_10sets.m
% 10セットのシミュレーション実行と解析を行うプログラム
% 全てのログとCSVはkalman\Resultsに保存される

function run_batch_10sets()
    % 前回のログとCSVを削除
    cleanup_previous_results();
    
    % プロジェクトルート
    proj_root = fileparts(mfilename('fullpath'));
    results_dir = fullfile(proj_root, 'Results');
    
    % Resultsディレクトリが存在しない場合は作成
    if ~exist(results_dir, 'dir')
        mkdir(results_dir);
    end
    
    % ログファイルを開く
    log_file = fullfile(results_dir, 'batch_10sets_log.txt');
    fid = fopen(log_file, 'w');
    
    fprintf(fid, '========================================\n');
    fprintf(fid, '10セット バッチシミュレーション開始\n');
    fprintf(fid, '開始時刻: %s\n', datestr(now));
    fprintf(fid, '========================================\n\n');
    fclose(fid);
    
    % 結果サマリー構造体
    results_summary = struct();
    
    % 10回実行
    for run_id = 1:10
        log_message(log_file, sprintf('\n========================================'));
        log_message(log_file, sprintf('=== Run %d/10 ===', run_id));
        log_message(log_file, sprintf('========================================'));
        
        try
            % シミュレーション実行
            log_message(log_file, sprintf('Run %d: シミュレーション開始...', run_id));
            tic;
            run_simulation(run_id, false);
            elapsed = toc;
            log_message(log_file, sprintf('Run %d: シミュレーション完了 (%.2f秒)', run_id, elapsed));
            
            % 結果ファイルを番号付きでコピー
            est_src = fullfile(proj_root, 'Results', 'estimation.csv');
            est_dst = fullfile(results_dir, sprintf('estimation_%02d.csv', run_id));
            copyfile(est_src, est_dst);
            log_message(log_file, sprintf('Run %d: 結果保存 -> %s', run_id, sprintf('estimation_%02d.csv', run_id)));
            
            % 解析実行
            log_message(log_file, sprintf('Run %d: 解析開始...', run_id));
            [metrics, has_error] = analyze_single_run(proj_root, run_id);
            
            if has_error
                log_message(log_file, sprintf('Run %d: エラー検出 - %s', run_id, metrics.error_msg));
                results_summary(run_id).status = 'FAILED';
                results_summary(run_id).error = metrics.error_msg;
                results_summary(run_id).pos_rmse = NaN;
            else
                % 結果保存
                results_summary(run_id).status = 'SUCCESS';
                results_summary(run_id).pos_rmse = metrics.pos_rmse;
                results_summary(run_id).vel_rmse = metrics.vel_rmse;
                results_summary(run_id).roll_rmse = metrics.roll_rmse;
                results_summary(run_id).pitch_rmse = metrics.pitch_rmse;
                results_summary(run_id).yaw_rmse = metrics.yaw_rmse;
                results_summary(run_id).ba_final = metrics.ba_final;
                results_summary(run_id).bg_final = metrics.bg_final;
                results_summary(run_id).max_innov = metrics.max_innov;
                results_summary(run_id).max_maha = metrics.max_maha;
                results_summary(run_id).has_nan = metrics.has_nan;
                results_summary(run_id).has_inf = metrics.has_inf;
                
                % ログ出力
                log_message(log_file, sprintf('Run %d Summary:', run_id));
                log_message(log_file, sprintf('  Position RMSE: %.4f m', metrics.pos_rmse));
                log_message(log_file, sprintf('  Velocity RMSE: %.4f m/s', metrics.vel_rmse));
                log_message(log_file, sprintf('  Roll/Pitch/Yaw RMSE: %.4f / %.4f / %.4f deg', ...
                    metrics.roll_rmse, metrics.pitch_rmse, metrics.yaw_rmse));
                log_message(log_file, sprintf('  Gyro bias (final): [%.4f, %.4f, %.4f] deg/s', ...
                    rad2deg(metrics.bg_final)));
                log_message(log_file, sprintf('  Max Innovation: %.4f', metrics.max_innov));
                log_message(log_file, sprintf('  Status: SUCCESS'));
            end
            
        catch e
            log_message(log_file, sprintf('Run %d FAILED: %s', run_id, e.message));
            results_summary(run_id).status = 'FAILED';
            results_summary(run_id).error = e.message;
            results_summary(run_id).pos_rmse = NaN;
        end
    end
    
    % 総合結果
    log_message(log_file, sprintf('\n========================================'));
    log_message(log_file, sprintf('=== 総合結果 ==='));
    log_message(log_file, sprintf('========================================'));
    
    success_count = 0;
    pos_rmse_all = [];
    vel_rmse_all = [];
    roll_rmse_all = [];
    pitch_rmse_all = [];
    yaw_rmse_all = [];
    
    for i = 1:10
        if strcmp(results_summary(i).status, 'SUCCESS')
            success_count = success_count + 1;
            pos_rmse_all = [pos_rmse_all, results_summary(i).pos_rmse];
            vel_rmse_all = [vel_rmse_all, results_summary(i).vel_rmse];
            roll_rmse_all = [roll_rmse_all, results_summary(i).roll_rmse];
            pitch_rmse_all = [pitch_rmse_all, results_summary(i).pitch_rmse];
            yaw_rmse_all = [yaw_rmse_all, results_summary(i).yaw_rmse];
        end
    end
    
    log_message(log_file, sprintf('成功: %d/10 (%.1f%%)', success_count, success_count*10));
    
    if success_count > 0
        log_message(log_file, sprintf('\n成功したRunの統計:'));
        log_message(log_file, sprintf('Position RMSE: Mean=%.4f, Std=%.4f, Max=%.4f m', ...
            mean(pos_rmse_all), std(pos_rmse_all), max(pos_rmse_all)));
        log_message(log_file, sprintf('Velocity RMSE: Mean=%.4f, Std=%.4f, Max=%.4f m/s', ...
            mean(vel_rmse_all), std(vel_rmse_all), max(vel_rmse_all)));
        log_message(log_file, sprintf('Roll RMSE: Mean=%.4f, Std=%.4f, Max=%.4f deg', ...
            mean(roll_rmse_all), std(roll_rmse_all), max(roll_rmse_all)));
        log_message(log_file, sprintf('Pitch RMSE: Mean=%.4f, Std=%.4f, Max=%.4f deg', ...
            mean(pitch_rmse_all), std(pitch_rmse_all), max(pitch_rmse_all)));
        log_message(log_file, sprintf('Yaw RMSE: Mean=%.4f, Std=%.4f, Max=%.4f deg', ...
            mean(yaw_rmse_all), std(yaw_rmse_all), max(yaw_rmse_all)));
    end
    
    log_message(log_file, sprintf('\n個別結果:'));
    for i = 1:10
        if strcmp(results_summary(i).status, 'SUCCESS')
            log_message(log_file, sprintf('Run %2d: PASS (Pos=%.4fm, Att=%.2f/%.2f/%.2f deg)', ...
                i, results_summary(i).pos_rmse, ...
                results_summary(i).roll_rmse, results_summary(i).pitch_rmse, results_summary(i).yaw_rmse));
        else
            if isfield(results_summary(i), 'error')
                log_message(log_file, sprintf('Run %2d: FAILED (%s)', i, results_summary(i).error));
            else
                log_message(log_file, sprintf('Run %2d: FAILED (Unknown error)', i));
            end
        end
    end
    
    % 総合判定
    if success_count == 10
        log_message(log_file, sprintf('\n=== 総合判定: 全てのRunが成功 ==='));
    elseif success_count >= 8
        log_message(log_file, sprintf('\n=== 総合判定: 良好 (80%%以上成功) ==='));
    elseif success_count >= 5
        log_message(log_file, sprintf('\n=== 総合判定: 要改善 (50%%以上成功) ==='));
    else
        log_message(log_file, sprintf('\n=== 総合判定: 不合格 (50%%未満成功) ==='));
    end
    
    % 結果をMATファイルに保存
    results_mat = fullfile(results_dir, 'batch_10sets_results.mat');
    save(results_mat, 'results_summary');
    log_message(log_file, sprintf('\nMAT結果保存: %s', 'batch_10sets_results.mat'));
    
    % CSVサマリー作成
    create_summary_csv(results_dir, results_summary);
    log_message(log_file, sprintf('CSVサマリー保存: %s', 'batch_10sets_summary.csv'));
    
    log_message(log_file, sprintf('\n終了時刻: %s', datestr(now)));
    log_message(log_file, sprintf('========================================'));
    
    fprintf('バッチ処理完了。ログ: %s\n', log_file);
end

function cleanup_previous_results()
    % 前回のログとCSVファイルを削除
    proj_root = fileparts(mfilename('fullpath'));
    results_dir = fullfile(proj_root, 'Results');
    
    if ~exist(results_dir, 'dir')
        return;
    end
    
    % 削除対象パターン
    patterns = {
        'estimation_*.csv',
        'batch_10sets_*.txt',
        'batch_10sets_*.mat',
        'batch_10sets_*.csv'
    };
    
    for i = 1:length(patterns)
        files = dir(fullfile(results_dir, patterns{i}));
        for j = 1:length(files)
            delete(fullfile(results_dir, files(j).name));
        end
    end
    
    fprintf('前回の結果を削除しました\n');
end

function log_message(log_file, msg)
    % ログファイルと標準出力に同時出力
    fprintf('%s\n', msg);
    fid = fopen(log_file, 'a');
    fprintf(fid, '%s\n', msg);
    fclose(fid);
end

function [metrics, has_error] = analyze_single_run(proj_root, run_id)
    % 単一Runの解析
    has_error = false;
    metrics = struct();
    
    try
        % データ読み込み
        est = readtable(fullfile(proj_root, 'Results', 'estimation.csv'));
        truth = readtable(fullfile(proj_root, 'GenerateData', 'truth_data.csv'));
        
        % 初期化期間後のインデックス (5秒 @ 400Hz = 2000サンプル)
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
        
        % 統計計算
        metrics.pos_rmse = rms(pos_err);
        metrics.vel_rmse = rms(vel_err);
        metrics.roll_rmse = rms(roll_err);
        metrics.pitch_rmse = rms(pitch_err);
        metrics.yaw_rmse = rms(yaw_err);
        
        % バイアス
        metrics.ba_final = [est.ba_x(end), est.ba_y(end), est.ba_z(end)];
        metrics.bg_final = [est.bg_x(end), est.bg_y(end), est.bg_z(end)];
        
        % 発散チェック
        metrics.max_innov = max(est.innov_norm);
        if ismember('maha_dist', est.Properties.VariableNames)
            metrics.max_maha = max(est.maha_dist);
        else
            metrics.max_maha = 0;  % maha_distカラムがない場合はデフォルト値
        end
        metrics.has_nan = any(isnan(est.px) | isnan(est.py) | isnan(est.pz));
        metrics.has_inf = any(isinf(est.px) | isinf(est.py) | isinf(est.pz));
        
        % エラーチェック
        if metrics.has_nan || metrics.has_inf
            has_error = true;
            metrics.error_msg = 'NaN/Inf detected';
        elseif metrics.pos_rmse > 1.0
            has_error = true;
            metrics.error_msg = sprintf('Position RMSE too high (%.2f m)', metrics.pos_rmse);
        end
        
    catch e
        has_error = true;
        metrics.error_msg = e.message;
    end
end

function create_summary_csv(results_dir, results_summary)
    % CSVサマリー作成
    fid = fopen(fullfile(results_dir, 'batch_10sets_summary.csv'), 'w');
    
    % ヘッダー
    fprintf(fid, 'Run,Status,Pos_RMSE_m,Vel_RMSE_ms,Roll_RMSE_deg,Pitch_RMSE_deg,Yaw_RMSE_deg,');
    fprintf(fid, 'BA_x,BA_y,BA_z,BG_x_deg,BG_y_deg,BG_z_deg,Max_Innov,Has_NaN,Has_Inf\n');
    
    % データ
    for i = 1:length(results_summary)
        fprintf(fid, '%d,%s,', i, results_summary(i).status);
        
        if strcmp(results_summary(i).status, 'SUCCESS')
            fprintf(fid, '%.4f,%.4f,%.4f,%.4f,%.4f,', ...
                results_summary(i).pos_rmse, results_summary(i).vel_rmse, ...
                results_summary(i).roll_rmse, results_summary(i).pitch_rmse, results_summary(i).yaw_rmse);
            fprintf(fid, '%.6f,%.6f,%.6f,', results_summary(i).ba_final);
            fprintf(fid, '%.6f,%.6f,%.6f,', rad2deg(results_summary(i).bg_final));
            fprintf(fid, '%.4f,%d,%d\n', ...
                results_summary(i).max_innov, results_summary(i).has_nan, results_summary(i).has_inf);
        else
            fprintf(fid, 'NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN\n');
        end
    end
    
    fclose(fid);
end
