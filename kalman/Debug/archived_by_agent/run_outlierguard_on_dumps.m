function run_outlierguard_on_dumps()
% RUN_OUTLIERGUARD_ON_DUMPS  Load divergence dumps and invoke OutlierGuard.checkAndApply
% for each dump to see whether the S inflation (S_minR_added) and related diagnostics
% are produced by the current OutlierGuard implementation.

% Navigate to repo root structure
cd(fileparts(mfilename('fullpath')));
% archived_by_agent sits under Debug/, so go two levels up to kalman/
base = fileparts(fileparts(pwd));
results = fullfile(base, 'Results');

% Ensure utility paths are available
addpath(fullfile(base, 'KF', 'Utils'));
addpath(fullfile(base, 'Debug'));

files = [dir(fullfile(results, 'divergence_full_dump_*.mat')); dir(fullfile(results, 'divergence_dump_*.mat'))];
n = length(files);
if n == 0
    fprintf('No divergence dump files found in %s\n', results);
    return;
end
cnt = 0;
for i = 1:n
    fn = fullfile(files(i).folder, files(i).name);
    try
        S = load(fn);
    catch e
        fprintf('%3d/%3d %s LOAD ERROR: %s\n', i, n, files(i).name, e.message);
        continue;
    end
    if isfield(S, 'dump')
        d = S.dump;
    else
        f = fieldnames(S);
        d = S.(f{1});
    end

    sensor = 'unknown'; if isfield(d, 'sensor'), sensor = lower(d.sensor); end
    z = [];
    if isfield(d, 'z'), z = d.z(:); end
    h = [];
    if isfield(d, 'h'), h = d.h(:); end

    if isfield(d, 'P'), P = d.P; elseif isfield(d, 'P_diag'), P = diag(d.P_diag); else P = eye(15); end
    if isfield(d, 'R'), R = d.R; elseif isfield(d, 'R_diag'), R = diag(d.R_diag); else R = [];
    end

    if strcmp(sensor, 'gps')
        H = [eye(3), zeros(3, 12)];
    elseif contains(sensor, 'mag')
        if ~isempty(h)
            try
                H = [zeros(3,6), quat_lib('skew', h), zeros(3,6)];
            catch
                H = zeros(length(h), 15);
            end
        else
            H = zeros(max(1, length(z)), 15);
        end
    else
        H = zeros(max(1,length(z)), 15);
    end

    % Create guards/estimators with defaults
    divergence_guard = DivergenceGuard(struct());
    noiseEstimator = NoiseEstimator(10);
    ctx = struct(); if isfield(d,'k'), ctx.k = d.k; end; ctx.z = z; ctx.h = h; if exist('P','var'), ctx.P_diag = diag(P); end; if exist('R','var') && ~isempty(R), ctx.R_diag = diag(R); end

    try
        [should_update, y_used, K_used, dx_used, diagnostics] = OutlierGuard.checkAndApply(sensor, z, h, H, P, R, [], [], divergence_guard, noiseEstimator, ctx);
    catch e
        fprintf('%3d/%3d %s ERROR running OutlierGuard: %s\n', i, n, files(i).name, e.message);
        continue;
    end

    smin = NaN; d2 = NaN; sr = NaN;
    if exist('diagnostics','var')
        if isfield(diagnostics, 'S_minR_added'), smin = diagnostics.S_minR_added; end
        if isfield(diagnostics, 'mahal_d2'), d2 = diagnostics.mahal_d2; end
        if isfield(diagnostics, 'S_rcond'), sr = diagnostics.S_rcond; end
    end

    fprintf('%3d/%3d %s sensor=%s should_update=%d S_minR_added=%g mahal_d2=%g S_rcond=%g\n', i, n, files(i).name, sensor, double(should_update), smin, d2, sr);
    if ~isnan(smin), cnt = cnt + 1; end
end

fprintf('SUMMARY: total=%d with_S_minR_added=%d\n', n, cnt);
end
