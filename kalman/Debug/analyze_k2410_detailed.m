% analyze_k2410_detailed.m
% Compare per-step log, full dumps, and raw sensor windows for target_k
try
    projRoot = fileparts(mfilename('fullpath'));
    resultsDir = fullfile(projRoot, '..', 'Results');
    dataDir = fullfile(projRoot, '..', 'GenerateData');
    target_k = 2410;

    % find latest debug_step_log
    debug_files = dir(fullfile(resultsDir, 'debug_step_log_*.csv'));
    if isempty(debug_files)
        error('No debug_step_log_*.csv in Results');
    end
    [~, idx] = max([debug_files.datenum]);
    debug_file = fullfile(resultsDir, debug_files(idx).name);
    T = readtable(debug_file);
    rows_k = T(T.k==target_k,:);
    fprintf('Found %d rows at k=%d in %s\n', height(rows_k), target_k, debug_file);
    disp(rows_k);

    % find full-dump MATs for k
    mat_files = dir(fullfile(resultsDir, sprintf('divergence_full_dump_*k%d*.mat', target_k)));
    fprintf('Found %d full-dump MAT files for k=%d\n', numel(mat_files), target_k);

    reports = [];
    for i=1:numel(mat_files)
        mf = fullfile(resultsDir, mat_files(i).name);
        s = load(mf);
        if ~isfield(s,'dump'), continue; end
        d = s.dump;
        rec = struct();
        rec.mat_file = mat_files(i).name;
        rec.sensor = '';
        if isfield(d,'sensor'), rec.sensor = d.sensor; end
        rec.k = target_k;
        rec.y_norm = NaN; rec.K_norm = NaN; rec.S_rcond = NaN;
        if isfield(d,'y') && ~isempty(d.y)
            rec.y_norm = norm(d.y);
        end
        if isfield(d,'K_approx') && ~isempty(d.K_approx)
            try
                rec.K_norm = norm(d.K_approx,'fro');
            catch
            end
        end
        if isfield(d,'S') && ~isempty(d.S)
            try
                rec.S_rcond = rcond(d.S);
            catch
            end
        end
        % raw window
        rec.raw_present = false;
        rec.mag_max_delta = NaN;
        rec.gps_alt_delta = NaN;
        if isfield(d,'raw_window') && ~isempty(d.raw_window)
            rec.raw_present = true;
            W = d.raw_window;
            % try to detect mag and gps changes if columns exist
            if all(ismember({'mx','my','mz'}, W.Properties.VariableNames))
                mx = W.mx; my = W.my; mz = W.mz;
                magvec = [mx,my,mz];
                deltas = abs(diff(magvec));
                rec.mag_max_delta = max(deltas(:));
            end
            if ismember('alt', W.Properties.VariableNames) && ismember('alt0', W.Properties.VariableNames)==0
                altv = W.alt;
                rec.gps_alt_delta = max(abs(diff(altv)));
            end
        else
            % try to read GenerateData/sensor_data.csv to extract ±5 samples
            sensor_file = fullfile(dataDir,'sensor_data.csv');
            if exist(sensor_file,'file')
                try
                    Sdata = readtable(sensor_file);
                    Nw = 5; i0 = max(1,target_k-Nw); i1 = min(height(Sdata), target_k+Nw);
                    W2 = Sdata(i0:i1,:);
                    if all(ismember({'mx','my','mz'}, W2.Properties.VariableNames))
                        mx=W2.mx; my=W2.my; mz=W2.mz; magvec=[mx,my,mz]; deltas=abs(diff(magvec)); rec.mag_max_delta=max(deltas(:)); end
                    if ismember('alt', W2.Properties.VariableNames)
                        rec.gps_alt_delta = max(abs(diff(W2.alt)));
                    end
                catch
                end
            end
        end

        reports = [reports; rec]; %#ok<AGROW>
    end

    if isempty(reports)
        warning('No full dump reports assembled');
    else
        Rtab = struct2table(reports);
        outname = fullfile(resultsDir, sprintf('analyze_k%d_detailed_report.csv', target_k));
        writetable(Rtab, outname);
        fprintf('Saved analysis report to %s\n', outname);
        disp(Rtab);
    end
catch e
    fprintf('ERROR in analyze_k2410_detailed: %s\n', e.message);
    disp(getReport(e));
end
