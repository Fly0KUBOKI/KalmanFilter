% analyze_debug_outputs.m
% Debug CSV を読み込み、統計量を計算し、プロットと要約を保存します.

function analyze_debug_outputs(write_all, run_sweep, sweep_write_csv)
    % analyze_debug_outputs(write_all, run_sweep)
    % If write_all is true, the function will also create per-file detail text
    % files and PNG plots. By default (no arg) write_all=false and only the
    % aggregated analysis_summary.txt is produced.
    % If run_sweep is true, a small parameter sweep will be executed first
    % to generate debug_output_*.csv files (then they are analyzed). Default
    % run_sweep=false.
    if nargin < 1 || isempty(write_all)
        write_all = false;
    end
    if nargin < 2 || isempty(run_sweep) 
        run_sweep = false;
    end
    if nargin < 3 || isempty(sweep_write_csv)
        sweep_write_csv = true; % backward compatible: default behavior writes CSVs
    end
    root = fileparts(mfilename('fullpath'));
    % if requested, run the small param sweep here (merged from param_sweep.m)
    if run_sweep
        try
            repoRoot = fullfile(root,'..','..');
            addpath(fullfile(repoRoot,'GenerateData'));
            addpath(fullfile(repoRoot,'KalmanFilter'));
            % prepare base params similar to param_sweep
            base_params = config_params();
            base_params.dt = 0.01;
            base_params.T = 30.0;
            base_params.kf.type = 'kf';
            base_params.kf.debug = false;
            base_params.kf.adaptive_R_enabled = false;
            base_params.data.source = 'csv';
            base_params.data.file = fullfile(repoRoot,'GenerateData','sim_data.csv');
            [~, meas_full, csvN] = load_sim_data(base_params.data.file);
            maxIter = min(floor(base_params.T/base_params.dt)+1, csvN);
            disable_steps_list = [0,1,3,5];
            infl_max_list = [10,50,100];
            sweepResults = containers.Map();
            for ds = disable_steps_list
                for im = infl_max_list
                    params = base_params;
                    params.kf.maha_disable_steps = ds;
                    params.kf.maha_inflation_max = im;
                    params.kf.maha_gate = 9;
                    outname = sprintf('debug_output_ds%02d_im%03d.csv', ds, im);
                    fprintf('analyze_debug_outputs: running sweep ds=%d im=%d -> %s\n', ds, im, outname);
                    if sweep_write_csv
                        run_trial(root, params, meas_full, maxIter, outname, true, false);
                        % load produced file into table for later analysis
                        try
                            T = readtable(fullfile(root, outname));
                            sweepResults(outname) = T;
                        catch ME
                            warning('Failed to read produced CSV %s: %s', outname, ME.message);
                        end
                    else
                        % run in-memory and store table
                        T = run_trial_collect(params, meas_full, maxIter, params.kf.init_from_first_gps, params.kf.maha_disable_steps>0);
                        sweepResults(outname) = T;
                    end
                end
            end
        catch ME
            warning(ME.identifier, '%s', ME.message);
        end
    end
    outDir = fullfile(root, 'plots');
    if ~exist(outDir, 'dir')
        mkdir(outDir);
    end

    % auto-detect debug_output_*.csv files in this folder
    listing = dir(fullfile(root, 'debug_output_*.csv'));
    files = cellfun(@(s) s.name, num2cell(listing), 'UniformOutput', false);
    if isempty(files)
        % fallback to known names for backward compatibility
        files = { 'debug_output_baseline.csv', 'debug_output_initgps_nogate.csv', 'debug_output_initgps.csv' };
    end

    gate_thresh = 9;
    summaries = cell(size(files));

    for i=1:numel(files)
        fname = files{i};
        % Prefer in-memory sweep results if present
        if exist('sweepResults','var') && isKey(sweepResults, fname)
            T = sweepResults(fname);
        else
            fpath = fullfile(root, fname);
            if ~exist(fpath, 'file')
                warning('File not found: %s', fpath);
                summaries{i} = struct('file', fname, 'error', 'not found');
                continue;
            end
            T = readtable(fpath);
        end
        % Expect columns: k,t,yg_x,yg_y,d2g,Sg_x,Sg_y,Ppos_x,Ppos_y,x_upd_x,x_upd_y
        % Defensive: check cols
        expected = {'k','t','yg_x','yg_y','d2g','Sg_x','Sg_y','Ppos_x','Ppos_y','x_upd_x','x_upd_y'};
        missing = setdiff(expected, T.Properties.VariableNames);
        if ~isempty(missing)
            warning('Missing columns in %s: %s', files{i}, strjoin(missing, ','));
        end

        % Basic stats
        nrows = height(T);
        initial_d2 = T.d2g(1);
        max_d2 = max(T.d2g);
        median_d2 = median(T.d2g);
        count_over = sum(T.d2g > gate_thresh);

        % x_upd norm and max
        xupd = [T.x_upd_x, T.x_upd_y];
        norms = sqrt(sum(xupd.^2, 2));
        [max_norm, idx_max] = max(norms);
        t_max = T.t(idx_max);

        % 1-step delta of x_upd (norm of change between consecutive updates)
        if height(T) >= 2 && all(ismember({'x_upd_x','x_upd_y'}, T.Properties.VariableNames))
            dx = sqrt(diff(T.x_upd_x).^2 + diff(T.x_upd_y).^2);
            [max_dx, idx_max_dx_rel] = max(dx);
            idx_max_dx = idx_max_dx_rel + 1; % dx(i) = change from i to i+1 -> report i+1 as step where jump observed
            t_max_dx = T.t(idx_max_dx);
        else
            dx = [];
            max_dx = NaN; idx_max_dx = NaN; t_max_dx = NaN;
        end

        summaries{i} = struct('file', files{i}, 'nrows', nrows, 'initial_d2', initial_d2, 'max_d2', max_d2, 'median_d2', median_d2, 'count_over', count_over, 'max_xupd_norm', max_norm, 'step_max_xupd', idx_max, 'time_max_xupd', t_max, 'max_delta_xupd', max_dx, 'step_max_delta', idx_max_dx, 'time_max_delta', t_max_dx);

        % Prepare per-file detailed event lists (top d^2 and top delta)
        try
            topN = 5;
            % top d^2 events
            valid_d2 = ~isnan(T.d2g);
            d2_idx = find(valid_d2);
            if ~isempty(d2_idx)
                [~, order] = sort(T.d2g(d2_idx),'descend');
                top_d2_idx = d2_idx(order(1:min(topN,numel(order))));
            else
                top_d2_idx = [];
            end

            % top delta events (dx corresponds to T rows 2:end)
            if exist('dx','var') && ~isempty(dx)
                [~, order_dx] = sort(dx,'descend');
                top_dx_idx = order_dx(1:min(topN,numel(order_dx))) + 1; % convert to T row index
            else
                top_dx_idx = [];
            end

            % write details file (optional)
            if write_all
                detailsName = fullfile(root, [files{i}(1:end-4) '_details.txt']);
                dfid = fopen(detailsName,'w');
                if dfid >= 0
                    fprintf(dfid, 'Details for %s\n\n', files{i});
                    fprintf(dfid, 'Top %d d^2 events (k,t,d2g,yg_x,yg_y,Sg_x,Sg_y,Ppos_x,Ppos_y,x_upd_x,x_upd_y)\n', topN);
                    for jj = 1:numel(top_d2_idx)
                        r = top_d2_idx(jj);
                        fprintf(dfid, '%d,%.3f,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g\n', T.k(r), T.t(r), T.d2g(r), T.yg_x(r), T.yg_y(r), T.Sg_x(r), T.Sg_y(r), T.Ppos_x(r), T.Ppos_y(r), T.x_upd_x(r), T.x_upd_y(r));
                    end
                    fprintf(dfid,'\nTop %d 1-step |Delta x_upd| events (k, t, delta, x_upd_prev_x, x_upd_prev_y, x_upd_x, x_upd_y)\n', topN);
                    for jj = 1:numel(top_dx_idx)
                        r = top_dx_idx(jj);
                        prev = r-1;
                        fprintf(dfid, '%d,%.3f,%.6g,%.6g,%.6g,%.6g,%.6g\n', T.k(r), T.t(r), sqrt((T.x_upd_x(r)-T.x_upd_x(prev))^2 + (T.x_upd_y(r)-T.x_upd_y(prev))^2), T.x_upd_x(prev), T.x_upd_y(prev), T.x_upd_x(r), T.x_upd_y(r));
                    end
                    fclose(dfid);
                end
            end
        catch ME
            warning('Failed to write details for %s: %s', files{i}, ME.message);
        end

            % Save time series plots (optional)
            if write_all
                try
                    fig = figure('visible','off');
                    subplot(4,1,1);
                    plot(T.t, T.d2g, '-'); hold on; yline(gate_thresh, '--r'); xlabel('t'); ylabel('d^2 (GPS)'); title(sprintf('%s - d^2', files{i}));

                    subplot(4,1,2);
                    plot(T.t, T.x_upd_x, '-', T.t, T.x_upd_y, '-'); xlabel('t'); ylabel('x'); legend('x_{upd}'); title('x_upd components');

                    subplot(4,1,3);
                    plot(T.t, T.Ppos_x, '-', T.t, T.Ppos_y, '-'); xlabel('t'); ylabel('P pos diag'); legend('P_x','P_y'); title('P position');

                    subplot(4,1,4);
                    if ~isempty(dx)
                        plot(T.t(2:end), dx, '-'); xlabel('t'); ylabel('Δ x_{upd} (m)'); title('1-step |Δ x_{upd}|'); hold on;
                        % use omitnan to avoid dependency on Statistics Toolbox
                        mm = mean(dx,'omitnan');
                        ss = std(dx,'omitnan');
                        yline(mm+3*ss,'--k');
                    else
                        text(0.1,0.5,'No x_upd data','Units','normalized');
                    end

                    saveas(fig, fullfile(outDir, [files{i} '.png']));
                    close(fig);
                catch ME
                    warning('Plot failed for %s: %s', files{i}, ME.message);
                end
            end
    end

    % Save summary text
    summaryFile = fullfile(root, 'analysis_summary.txt');
    fid = fopen(summaryFile, 'w');
    if fid < 0
        warning('Cannot open summary file for writing: %s', summaryFile);
        return;
    end
    fprintf(fid, 'Analysis summary\n');
    fprintf(fid, 'Gate threshold = %g\n\n', gate_thresh);
    for i=1:numel(summaries)
        s = summaries{i};
        if isfield(s, 'error')
            fprintf(fid, '%s: ERROR: %s\n\n', s.file, s.error);
            continue;
        end
        fprintf(fid, 'File: %s\n', s.file);
        fprintf(fid, ' Rows: %d\n', s.nrows);
        fprintf(fid, ' Initial d^2: %.6g\n', s.initial_d2);
        fprintf(fid, ' Max d^2: %.6g\n', s.max_d2);
        fprintf(fid, ' Median d^2: %.6g\n', s.median_d2);
        fprintf(fid, ' Count d^2 > %g: %d\n', gate_thresh, s.count_over);
        fprintf(fid, ' Max |x_upd|: %.6g at step %d (t=%.3f)\n', s.max_xupd_norm, s.step_max_xupd, s.time_max_xupd);
        % 1-step delta summary (if available)
        if isfield(s,'max_delta_xupd') && ~isnan(s.max_delta_xupd)
            fprintf(fid, ' Max 1-step |Δ x_upd|: %.6g at step %d (t=%.3f)\n', s.max_delta_xupd, s.step_max_delta, s.time_max_delta);
        else
            fprintf(fid, ' Max 1-step |\Delta x_upd|: N/A\n');
        end
        fprintf(fid, '\n');
    end
    fclose(fid);
    fprintf('Analysis complete. Summary saved to %s\n', summaryFile);
end
