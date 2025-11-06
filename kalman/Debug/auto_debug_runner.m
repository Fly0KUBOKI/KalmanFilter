function auto_debug_runner(varargin)
% AUTO_DEBUG_RUNNER  自動でフィルタを実行して詳細ステップログを収集する
%
%  auto_debug_runner() - シミュレーションデータを生成し、ESKF を実行して
%                       各ステップの P, innovation, K_norm 等を Results に保存する

params = struct();
params.run_simulation = true;
params.outdir = fullfile(fileparts(mfilename('fullpath')), '..', 'Results');
params.save_csv = true;
if ~isempty(varargin) && isstruct(varargin{1})
    f = fieldnames(varargin{1});
    for i=1:numel(f)
        params.(f{i}) = varargin{1}.(f{i});
    end
end

projRoot = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(projRoot, '..')));
rehash;

if params.run_simulation
    try
        fprintf('Running run_simulation (auto_debug_runner)...\n');
        run_simulation();
    catch e
        warning('auto_debug_runner:run_simulation', 'run_simulation failed: %s', e.message);
    end
end

% load observations
dataDir = fullfile(projRoot, '..', 'GenerateData');
obsFile = fullfile(dataDir, 'sensor_data.csv');
if ~exist(obsFile,'file')
    error('sensor_data.csv not found: %s', obsFile);
end
obs = read_csv(obsFile);

dt = mean(diff(obs.time));
eskf = ESKF(obs, 2.0, dt);

% prepare logs
logs = struct('k', {}, 'time', {}, 'stage', {}, 'max_diag', {}, 'traceP', {}, 'rcondP', {}, 'K_norm', {}, 'y_norm', {}, 'z_norm', {});

    function appendLog(info)
        % nested function to capture logs
        entry = struct();
        if isfield(info, 'k'), entry.k = info.k; else entry.k = NaN; end
        if isfield(info, 'p'), entry.time = NaN; else entry.time = NaN; end
        if isfield(info, 'stage')
            entry.stage = info.stage;
        else
            entry.stage = 'unknown';
        end

        if isfield(info, 'P')
            P = info.P;
            d = diag(P);
            entry.max_diag = max(d);
            entry.traceP = trace(P);
            entry.rcondP = rcond(P);
        elseif isfield(info, 'P_diag')
            d = info.P_diag;
            entry.max_diag = max(d);
            entry.traceP = sum(d);
            entry.rcondP = NaN;
        else
            entry.max_diag = NaN;
            entry.traceP = NaN;
            entry.rcondP = NaN;
        end

        if isfield(info, 'K_norm')
            entry.K_norm = info.K_norm;
        else
            entry.K_norm = NaN;
        end

        if isfield(info, 'y')
            entry.y_norm = norm(info.y);
        else
            entry.y_norm = NaN;
        end

        if isfield(info, 'z')
            entry.z_norm = norm(info.z);
        else
            entry.z_norm = NaN;
        end
        logs(end+1) = entry; %#ok<AGROW>
    end

% attach callback
eskf.debugCallback = @appendLog;

N = numel(obs.time);
fprintf('Running ESKF for %d steps and collecting debug logs...\n', N);
for k=1:N
    eskf.updateFilter(obs, k);
end

% save logs
if ~exist(params.outdir, 'dir'), mkdir(params.outdir); end
ts = datestr(now, 'yyyymmdd_HHMMSS');
matfile = fullfile(params.outdir, sprintf('debug_step_log_%s.mat', ts));
save(matfile, 'logs');
fprintf('Saved debug logs to %s\n', matfile);

if params.save_csv
    % convert to table with selected scalar columns
    T = struct2table(logs);
    csvfile = fullfile(params.outdir, sprintf('debug_step_log_%s.csv', ts));
    try
        writetable(T, csvfile);
        fprintf('Saved debug CSV to %s\n', csvfile);
    catch
        warning('Could not write CSV (some fields may be non-scalar). Saved MAT only.');
    end
end

fprintf('auto_debug_runner finished. %d log rows saved.\n', numel(logs));
end
