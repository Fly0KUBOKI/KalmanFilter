% run_batch_10sets.m
% 10セットのシミュレーション実行と解析を行うプログラム
% ログは `kalman/Results/log` に、CSV 等の結果は `kalman/Results` に保存される（ログは上書きされません）

function run_batch_10sets(use_mex)
    % run_batch_10sets(use_mex)
    %  use_mex (optional, default=false) : true にすると MEX 実装を優先して実行します
    if nargin < 1 || isempty(use_mex)
        % If the environment variable FORCE_MATLAB_FILTERS is set, respect it.
        % FORCE_MATLAB_FILTERS = '1' -> use MATLAB filters (use_mex = false)
        % FORCE_MATLAB_FILTERS = '0' -> use MEX filters (use_mex = true)
        env_force = getenv('FORCE_MATLAB_FILTERS');
        if ~isempty(env_force)
            if strcmp(env_force, '1')
                use_mex = false;
            else
                use_mex = true;
            end
        else
            use_mex = false;
        end
    end
    % 前回のログとCSVを削除
    cleanup_previous_results();
    
    % プロジェクトルート
    proj_root = fileparts(mfilename('fullpath'));
    results_dir = fullfile(proj_root, 'Results');
    
    % Resultsディレクトリが存在しない場合は作成
    if ~exist(results_dir, 'dir')
        mkdir(results_dir);
    end

    % C++ MEX バイナリのパスを追加（bin のみ）
    clear mex;  % 古い MEX キャッシュを明示的にクリア
    mex_bin = fullfile(proj_root, 'cpp', 'bin');
    if exist(mex_bin, 'dir')
        addpath(mex_bin);
    end
 
    addpath(genpath(fullfile(proj_root, 'ESKF')));
    addpath(fullfile(proj_root, 'Graph'));
    addpath(fullfile(proj_root, 'GenerateData'));
        if exist('mex_sensor_filter','file') == 3
            % 明示的ゼロ初期化を優先
            try
                SensorFilters.reset_zero();
            catch
                SensorFilters.reset();
            end
        end
    
    % MEX / MATLAB フィルタ選択
    if use_mex
        setenv('FORCE_MATLAB_FILTERS', '0');
    else
        setenv('FORCE_MATLAB_FILTERS', '1');
    end
    
    % ログディレクトリを作成（ログはここに集約し、タイムスタンプ付きで上書きしない）
    log_dir = fullfile(results_dir, 'log');
    if ~exist(log_dir, 'dir')
        mkdir(log_dir);
    end
    % モードラベル（MEX or MATLAB）とタイムスタンプを付与して一意化
    if use_mex
        mode_label = 'mex';
    else
        mode_label = 'matlab';
    end
    tstr = datestr(now, 'yyyymmdd_HHMMSS');
    log_file = fullfile(log_dir, sprintf('batch_10sets_log_%s_%s.txt', mode_label, tstr));
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
            
            % 判定: 各軸ごとにRMSE閾値で判定する
            thr = 1.0; % 位置RMSE閾値 (m)
            att_thr = 5.0; % 姿勢RMSE閾値 (deg) - Roll/Pitch/Yaw がこの値を超えるとFAILED
            % 基本フィールドは常に保存
            results_summary(run_id).pos_rmse = metrics.pos_rmse;
            results_summary(run_id).posx_rmse = metrics.posx_rmse;
            results_summary(run_id).posy_rmse = metrics.posy_rmse;
            results_summary(run_id).posz_rmse = metrics.posz_rmse;
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

            % NaN/Infがある場合は全軸FAILED
            if metrics.has_nan || metrics.has_inf
                results_summary(run_id).posx_ok = false;
                results_summary(run_id).posy_ok = false;
                results_summary(run_id).posz_ok = false;
                results_summary(run_id).status = 'FAILED';
                results_summary(run_id).error = metrics.error_msg;
                log_message(log_file, sprintf('Run %d: エラー検出 - %s', run_id, metrics.error_msg));
            else
                % 軸別判定（位置）
                posx_ok = metrics.posx_rmse <= thr;
                posy_ok = metrics.posy_rmse <= thr;
                posz_ok = metrics.posz_rmse <= thr;
                results_summary(run_id).posx_ok = posx_ok;
                results_summary(run_id).posy_ok = posy_ok;
                results_summary(run_id).posz_ok = posz_ok;

                % 姿勢判定（Roll/Pitch/Yaw）
                att_ok = (metrics.roll_rmse <= att_thr) && (metrics.pitch_rmse <= att_thr) && (metrics.yaw_rmse <= att_thr);
                results_summary(run_id).att_ok = att_ok;

                % 追加条件: 各姿勢軸の厳密閾値（deg）
                att_axis_thr = 1.0; % Roll/Pitch/Yaw 各軸の厳格閾値 (deg)
                att_axis_ok = (metrics.roll_rmse <= att_axis_thr) && (metrics.pitch_rmse <= att_axis_thr) && (metrics.yaw_rmse <= att_axis_thr);
                results_summary(run_id).att_axis_ok = att_axis_ok;

                % 全軸（位置）かつ姿勢合格かつ姿勢各軸が閾値以下ならSUCCESS, そうでなければFAILED
                if posx_ok && posy_ok && posz_ok && att_ok && att_axis_ok
                    results_summary(run_id).status = 'SUCCESS';
                    log_message(log_file, sprintf('Run %d Summary: PASS (Position axes within %.2fm and attitude within %.1fdeg; attitude axes within %.1fdeg)', run_id, thr, att_thr, att_axis_thr));
                else
                    results_summary(run_id).status = 'FAILED';
                    % どの軸がFailか列挙
                    failed = {};
                    if ~posx_ok, failed{end+1} = sprintf('X(%.2f)', metrics.posx_rmse); end
                    if ~posy_ok, failed{end+1} = sprintf('Y(%.2f)', metrics.posy_rmse); end
                    if ~posz_ok, failed{end+1} = sprintf('Z(%.2f)', metrics.posz_rmse); end
                    if ~att_ok
                        failed{end+1} = sprintf('Att(R=%.2f,P=%.2f,Y=%.2f)', metrics.roll_rmse, metrics.pitch_rmse, metrics.yaw_rmse);
                    end
                    results_summary(run_id).error = sprintf('Axis RMSE too high: %s', strjoin(failed, ', '));
                    log_message(log_file, sprintf('Run %d: エラー検出 - %s', run_id, results_summary(run_id).error));
                end

                % ログ出力の詳細（常に表示）
                log_message(log_file, sprintf('  Position RMSE: Overall=%.4f m, X=%.4f m, Y=%.4f m, Z=%.4f m', ...
                    metrics.pos_rmse, metrics.posx_rmse, metrics.posy_rmse, metrics.posz_rmse));
                log_message(log_file, sprintf('  Velocity RMSE: %.4f m/s', metrics.vel_rmse));
                log_message(log_file, sprintf('  Roll/Pitch/Yaw RMSE: %.4f / %.4f / %.4f deg', ...
                    metrics.roll_rmse, metrics.pitch_rmse, metrics.yaw_rmse));
                log_message(log_file, sprintf('  Gyro bias (final): [%.4f, %.4f, %.4f] deg/s', ...
                    rad2deg(metrics.bg_final)));
                log_message(log_file, sprintf('  Max Innovation: %.4f', metrics.max_innov));
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
    posx_rmse_all = [];
    posy_rmse_all = [];
    posz_rmse_all = [];
    vel_rmse_all = [];
    roll_rmse_all = [];
    pitch_rmse_all = [];
    yaw_rmse_all = [];
    
    for i = 1:10
        if strcmp(results_summary(i).status, 'SUCCESS')
            success_count = success_count + 1;
            pos_rmse_all = [pos_rmse_all, results_summary(i).pos_rmse];
            posx_rmse_all = [posx_rmse_all, results_summary(i).posx_rmse];
            posy_rmse_all = [posy_rmse_all, results_summary(i).posy_rmse];
            posz_rmse_all = [posz_rmse_all, results_summary(i).posz_rmse];
            vel_rmse_all = [vel_rmse_all, results_summary(i).vel_rmse];
            roll_rmse_all = [roll_rmse_all, results_summary(i).roll_rmse];
            pitch_rmse_all = [pitch_rmse_all, results_summary(i).pitch_rmse];
            yaw_rmse_all = [yaw_rmse_all, results_summary(i).yaw_rmse];
        end
    end
    
    log_message(log_file, sprintf('成功: %d/10 (%.1f%%)', success_count, success_count*10));
    
    if success_count > 0
        log_message(log_file, sprintf('\n成功したRunの統計:'));
        log_message(log_file, sprintf('Position RMSE (overall): Mean=%.4f, Std=%.4f, Max=%.4f m', ...
            mean(pos_rmse_all), std(pos_rmse_all), max(pos_rmse_all)));
        log_message(log_file, sprintf('Position RMSE by axis: X Mean=%.4f, Y Mean=%.4f, Z Mean=%.4f m', ...
            mean(posx_rmse_all), mean(posy_rmse_all), mean(posz_rmse_all)));
        log_message(log_file, sprintf('Position RMSE by axis: X Std=%.4f, Y Std=%.4f, Z Std=%.4f m', ...
            std(posx_rmse_all), std(posy_rmse_all), std(posz_rmse_all)));
        log_message(log_file, sprintf('Position RMSE by axis: X Max=%.4f, Y Max=%.4f, Z Max=%.4f m', ...
            max(posx_rmse_all), max(posy_rmse_all), max(posz_rmse_all)));
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
                log_message(log_file, sprintf('Run %2d: PASS (Pos Overall=%.4fm, X=%.4fm, Y=%.4fm, Z=%.4fm, Att=%.2f/%.2f/%.2f deg)', ...
                i, results_summary(i).pos_rmse, results_summary(i).posx_rmse, results_summary(i).posy_rmse, results_summary(i).posz_rmse, ...
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
        
        % 誤差計算（軸別および総合）
        posx_err = (est.px(idx) - truth.x(idx));
        posy_err = (est.py(idx) - truth.y(idx));
        posz_err = (est.pz(idx) - truth.z(idx));
        pos_err = sqrt(posx_err.^2 + posy_err.^2 + posz_err.^2);
        vel_err = sqrt((est.vx(idx) - truth.vx(idx)).^2 + ...
                       (est.vy(idx) - truth.vy(idx)).^2 + ...
                       (est.vz(idx) - truth.vz(idx)).^2);
        
        roll_err = rad2deg(wrapToPi(deg2rad(est.roll(idx) - truth.roll(idx))));
        pitch_err = rad2deg(wrapToPi(deg2rad(est.pitch(idx) - truth.pitch(idx))));
        yaw_err = rad2deg(wrapToPi(deg2rad(est.yaw(idx) - truth.yaw(idx))));
        
        % 統計計算
        metrics.pos_rmse = rms(pos_err);
        metrics.posx_rmse = rms(posx_err);
        metrics.posy_rmse = rms(posy_err);
        metrics.posz_rmse = rms(posz_err);
        metrics.vel_rmse = rms(vel_err);
        metrics.roll_rmse = rms(roll_err);
        metrics.pitch_rmse = rms(pitch_err);
        metrics.yaw_rmse = rms(yaw_err);
        
        % バイアス
        metrics.ba_final = [est.ba_x(end), est.ba_y(end), est.ba_z(end)];
        metrics.bg_final = [est.bg_x(end), est.bg_y(end), est.bg_z(end)];
        
        % 発散チェック
        if ismember('innov_norm', est.Properties.VariableNames)
            metrics.max_innov = max(est.innov_norm);
        else
            metrics.max_innov = 0;  % innov_normカラムがない場合はデフォルト値
        end
        if ismember('maha_dist', est.Properties.VariableNames)
            metrics.max_maha = max(est.maha_dist);
        else
            metrics.max_maha = 0;  % maha_distカラムがない場合はデフォルト値
        end
        metrics.has_nan = any(isnan(est.px) | isnan(est.py) | isnan(est.pz));
        metrics.has_inf = any(isinf(est.px) | isinf(est.py) | isinf(est.pz));
        
        % エラーチェック（NaN/Inf と軸別発散判定）
        if metrics.has_nan || metrics.has_inf
            has_error = true;
            metrics.error_msg = 'NaN/Inf detected';
        else
            % 軸別しきい値 (m)
            thr = 1.0;
            bad_axes = {};
            if metrics.posx_rmse > thr
                bad_axes{end+1} = sprintf('X(%.2f)', metrics.posx_rmse);
            end
            if metrics.posy_rmse > thr
                bad_axes{end+1} = sprintf('Y(%.2f)', metrics.posy_rmse);
            end
            if metrics.posz_rmse > thr
                bad_axes{end+1} = sprintf('Z(%.2f)', metrics.posz_rmse);
            end

            if ~isempty(bad_axes)
                has_error = true;
                metrics.error_msg = sprintf('Position RMSE too high on axis: %s', strjoin(bad_axes, ', '));
            elseif metrics.pos_rmse > thr
                % 全体RMSEが閾値超過だが軸別RMSEはしきい値未満の場合、
                % 二乗和から各軸の寄与率を計算して最大寄与軸を報告する
                total_sq = metrics.posx_rmse^2 + metrics.posy_rmse^2 + metrics.posz_rmse^2;
                if total_sq > 0
                    frac = [metrics.posx_rmse^2, metrics.posy_rmse^2, metrics.posz_rmse^2] ./ total_sq * 100;
                    [max_frac, idx_max] = max(frac);
                    axis_names = {'X','Y','Z'};
                    has_error = true;
                    metrics.error_msg = sprintf('Position RMSE too high (overall %.2f m); largest contributor %s (%.1f%%)', ...
                        metrics.pos_rmse, axis_names{idx_max}, max_frac);
                else
                    has_error = true;
                    metrics.error_msg = sprintf('Position RMSE too high (overall %.2f m)', metrics.pos_rmse);
                end
            end
        end
        
    catch e
        has_error = true;
        metrics.error_msg = e.message;
    end
end

function create_summary_csv(results_dir, results_summary)
    % CSVサマリー作成（軸別RMSEとOKフラグを含む）
    fid = fopen(fullfile(results_dir, 'batch_10sets_summary.csv'), 'w');

    % ヘッダー
    fprintf(fid, 'Run,Status,PosX_RMSE_m,PosY_RMSE_m,PosZ_RMSE_m,PosX_OK,PosY_OK,PosZ_OK,Vel_RMSE_ms,Roll_RMSE_deg,Pitch_RMSE_deg,Yaw_RMSE_deg,');
    fprintf(fid, 'BA_x,BA_y,BA_z,BG_x_deg,BG_y_deg,BG_z_deg,Max_Innov,Has_NaN,Has_Inf\n');

    % データ
    for i = 1:length(results_summary)
        s = results_summary(i);
        fprintf(fid, '%d,%s,', i, s.status);

        % Safe access to fields (some FAILED runs may miss fields)
        posx = NaN; posy = NaN; posz = NaN; posx_ok = 0; posy_ok = 0; posz_ok = 0;
        vel = NaN; roll = NaN; pitch = NaN; yaw = NaN;
        ba = [NaN, NaN, NaN]; bg = [NaN, NaN, NaN]; max_innov = NaN; has_nan = 0; has_inf = 0;

        if isfield(s, 'posx_rmse'), posx = s.posx_rmse; end
        if isfield(s, 'posy_rmse'), posy = s.posy_rmse; end
        if isfield(s, 'posz_rmse'), posz = s.posz_rmse; end
        if isfield(s, 'posx_ok'), posx_ok = double(s.posx_ok); end
        if isfield(s, 'posy_ok'), posy_ok = double(s.posy_ok); end
        if isfield(s, 'posz_ok'), posz_ok = double(s.posz_ok); end
        if isfield(s, 'vel_rmse'), vel = s.vel_rmse; end
        if isfield(s, 'roll_rmse'), roll = s.roll_rmse; end
        if isfield(s, 'pitch_rmse'), pitch = s.pitch_rmse; end
        if isfield(s, 'yaw_rmse'), yaw = s.yaw_rmse; end
        if isfield(s, 'ba_final'), ba = s.ba_final; end
        if isfield(s, 'bg_final'), bg = rad2deg(s.bg_final); end
        if isfield(s, 'max_innov'), max_innov = s.max_innov; end
        if isfield(s, 'has_nan'), has_nan = double(s.has_nan); end
        if isfield(s, 'has_inf'), has_inf = double(s.has_inf); end

        fprintf(fid, '%.4f,%.4f,%.4f,%d,%d,%d,%.4f,%.4f,%.4f,%.4f,', ...
            posx, posy, posz, posx_ok, posy_ok, posz_ok, vel, roll, pitch, yaw);
        fprintf(fid, '%.6f,%.6f,%.6f,', ba);
        fprintf(fid, '%.6f,%.6f,%.6f,', bg);
        fprintf(fid, '%.4f,%d,%d\n', max_innov, has_nan, has_inf);
    end

    fclose(fid);
end
