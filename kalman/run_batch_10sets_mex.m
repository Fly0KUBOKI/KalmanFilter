% run_batch_10sets_mex.m
% run 10 simulation sets but force use of C++ MEX sensor filters (FORCE_MATLAB_FILTERS=0)
function run_batch_10sets_mex()
    % 前回のログとCSVを削除 (inline cleanup to avoid dependency on run_batch_10sets)
    proj_root = fileparts(mfilename('fullpath'));
    results_dir = fullfile(proj_root, 'Results');
    if exist(results_dir, 'dir')
        patterns = {'estimation_*.csv', 'batch_10sets_*.txt', 'batch_10sets_*.mat', 'batch_10sets_*.csv'};
        for i = 1:length(patterns)
            files = dir(fullfile(results_dir, patterns{i}));
            for j = 1:length(files)
                delete(fullfile(results_dir, files(j).name));
            end
        end
        fprintf('前回の結果を削除しました\n');
    else
        mkdir(results_dir);
    end

    % Add mex bin if exists
    mex_bin = fullfile(proj_root, 'cpp', 'bin');
    if exist(mex_bin, 'dir'), addpath(mex_bin); end

    % Reset MEX state if present
    try
        clear mex;
        if exist('mex_sensor_filter','file') == 3
            try
                SensorFilters.reset_zero();
            catch
                SensorFilters.reset();
            end
        end
    catch
    end

    % Force use of MEX filters
    setenv('FORCE_MATLAB_FILTERS', '0');

    % Use same logging/analysis flow as run_batch_10sets
    log_file = fullfile(results_dir, 'batch_10sets_mex_log.txt');
    fid = fopen(log_file, 'w'); fclose(fid);
    log_message(log_file, '========================================');
    log_message(log_file, '10セット バッチシミュレーション (MEX経路) 開始');
    log_message(log_file, sprintf('開始時刻: %s', datestr(now)));

    results_summary = struct();
    for run_id = 1:10
        log_message(log_file, sprintf('\n========================================'));
        log_message(log_file, sprintf('=== Run %d/10 ===', run_id));
        log_message(log_file, sprintf('========================================'));
        try
            log_message(log_file, sprintf('Run %d: シミュレーション開始...', run_id));
            tic;
            run_simulation(run_id, false);
            elapsed = toc;
            log_message(log_file, sprintf('Run %d: シミュレーション完了 (%.2f秒)', run_id, elapsed));
            est_src = fullfile(proj_root, 'Results', 'estimation.csv');
            est_dst = fullfile(results_dir, sprintf('estimation_mex_%02d.csv', run_id));
            copyfile(est_src, est_dst);
            % Also copy to workspace root Results for easy top-level comparison
            try
                root_results = fullfile(fileparts(proj_root), 'Results');
                if ~exist(root_results, 'dir'), mkdir(root_results); end
                est_root_dst = fullfile(root_results, sprintf('estimation_mex_%02d.csv', run_id));
                copyfile(est_src, est_root_dst);
                log_message(log_file, sprintf('Run %d: 結果保存 -> %s and %s', run_id, sprintf('estimation_mex_%02d.csv', run_id), fullfile('Results', sprintf('estimation_mex_%02d.csv', run_id))));
            catch e
                % fallback: log but continue
                log_message(log_file, sprintf('Run %d: ルートResultsにコピーできませんでした: %s', run_id, e.message));
            end

            % Simple analysis: reuse analyze_single_run code inline
            [metrics, has_error] = analyze_single_run_for_wrapper(proj_root, run_id, est_dst);

            % store minimal summary
            results_summary(run_id).pos_rmse = metrics.pos_rmse;
            results_summary(run_id).posx_rmse = metrics.posx_rmse;
            results_summary(run_id).posy_rmse = metrics.posy_rmse;
            results_summary(run_id).posz_rmse = metrics.posz_rmse;
            results_summary(run_id).vel_rmse = metrics.vel_rmse;
            results_summary(run_id).roll_rmse = metrics.roll_rmse;
            results_summary(run_id).pitch_rmse = metrics.pitch_rmse;
            results_summary(run_id).yaw_rmse = metrics.yaw_rmse;
            if has_error && isfield(metrics,'error_msg') && ~isempty(metrics.error_msg) && ~strcmp(metrics.error_msg,'')
                results_summary(run_id).status = 'FAILED';
            else
                results_summary(run_id).status = 'SUCCESS';
            end
            if isfield(metrics,'error_msg')
                results_summary(run_id).error = metrics.error_msg;
            end

        catch e
            log_message(log_file, sprintf('Run %d FAILED: %s', run_id, e.message));
            results_summary(run_id).status = 'FAILED';
        end
    end

    % Save summary and CSV
    save(fullfile(results_dir,'batch_10sets_mex_results.mat'),'results_summary');
    create_summary_csv(results_dir, results_summary);
    log_message(log_file, sprintf('\n終了時刻: %s', datestr(now)));
    log_message(log_file, '========================================');
    fprintf('MEXバッチ完了。ログ: %s\n', log_file);
end

function [metrics, has_error] = analyze_single_run_for_wrapper(proj_root, run_id, est_file)
    has_error = false; metrics = struct();
    try
        est = readtable(est_file);
        truth = readtable(fullfile(proj_root,'GenerateData','truth_data.csv'));
        init_samples = 2000;
        idx = init_samples+1:height(est);
        posx_err = (est.px(idx) - truth.x(idx));
        posy_err = (est.py(idx) - truth.y(idx));
        posz_err = (est.pz(idx) - truth.z(idx));
        pos_err = sqrt(posx_err.^2 + posy_err.^2 + posz_err.^2);
        vel_err = sqrt((est.vx(idx) - truth.vx(idx)).^2 + (est.vy(idx) - truth.vy(idx)).^2 + (est.vz(idx) - truth.vz(idx)).^2);
        roll_err = rad2deg(wrapToPi(deg2rad(est.roll(idx) - truth.roll(idx))));
        pitch_err = rad2deg(wrapToPi(deg2rad(est.pitch(idx) - truth.pitch(idx))));
        yaw_err = rad2deg(wrapToPi(deg2rad(est.yaw(idx) - truth.yaw(idx))));
        metrics.pos_rmse = rms(pos_err);
        metrics.posx_rmse = rms(posx_err); metrics.posy_rmse = rms(posy_err); metrics.posz_rmse = rms(posz_err);
        metrics.vel_rmse = rms(vel_err);
        metrics.roll_rmse = rms(roll_err); metrics.pitch_rmse = rms(pitch_err); metrics.yaw_rmse = rms(yaw_err);
        metrics.ba_final = [est.ba_x(end), est.ba_y(end), est.ba_z(end)];
        metrics.bg_final = [est.bg_x(end), est.bg_y(end), est.bg_z(end)];
        metrics.has_nan = any(isnan(est.px) | isnan(est.py) | isnan(est.pz));
        metrics.has_inf = any(isinf(est.px) | isinf(est.py) | isinf(est.pz));
        if metrics.has_nan || metrics.has_inf
            has_error = true; metrics.error_msg = 'NaN/Inf detected';
        elseif metrics.posx_rmse > 1 || metrics.posy_rmse > 1 || metrics.posz_rmse > 1
            has_error = true; metrics.error_msg = 'Position RMSE too high on axis';
        else
            has_error = false; metrics.error_msg = '';
        end
    catch e
        has_error = true; metrics.error_msg = e.message;
    end
end

function log_message(log_file, msg)
    % ログファイルと標準出力に同時出力
    fprintf('%s\n', msg);
    fid = fopen(log_file, 'a');
    if fid ~= -1
        fprintf(fid, '%s\n', msg);
        fclose(fid);
    end
end
