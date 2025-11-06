function apply_mag_preprocessor_on_dumps()
% APPLY_MAG_PREPROCESSOR_ON_DUMPS  Apply Hampel+EMA preprocessing to mag dumps
% and re-run OutlierGuard.checkAndApply to compare diagnostics before/after.

cd(fileparts(mfilename('fullpath')));
base = fileparts(pwd); results = fullfile(base,'Results');
addpath(fullfile(base,'KF','Utils'));
addpath(fullfile(base,'KF','Utils','..'));

files = [dir(fullfile(results,'divergence_dump_mag_*.mat')); dir(fullfile(results,'divergence_full_dump_mag_*.mat'))];
if isempty(files)
    fprintf('No mag divergence dumps found in %s\n', results);
    return;
end

for i=1:length(files)
    fn = fullfile(files(i).folder, files(i).name);
    S = load(fn);
    if isfield(S,'dump'), d = S.dump; else f=fieldnames(S); d=S.(f{1}); end
    fprintf('\n=== %s ===\n', files(i).name);
    z_orig = [];
    if isfield(d,'z'), z_orig = d.z(:); end

    % compute cleaned measurement using raw_window if present
    z_clean = z_orig;
    if isfield(d,'raw_window') && ~isempty(d.raw_window)
        % take last row as new sample, previous as buffer
        W = d.raw_window;
        last = W(end,:)';
        prev = W(1:end-1,:)'; % columns are samples
        if isempty(prev)
            buf = [];
        else
            buf = prev(:);
        end
        % apply Hampel per component
        zc = zeros(size(last));
        for k=1:numel(last)
            try
                zc(k) = hampel_causal(buf(k:numel(last):end), last(k), 5, 3);
            catch
                zc(k) = last(k);
            end
        end
        % then EMA with alpha=0.1 using median(prev) as previous state
        if ~isempty(prev)
            prev_med = median(prev,2);
            zc2 = zeros(size(zc));
            for k=1:numel(zc)
                zc2(k) = ema_update(zc(k), prev_med(k), 0.1);
            end
            z_clean = zc2;
        else
            z_clean = zc;
        end
    else
        % no raw window: apply simple Hampel with empty buffer
        try
            z_clean = hampel_causal([], z_orig, 5, 3);
        catch
            z_clean = z_orig;
        end
    end

    % Build H/P/R like run_outlierguard_on_dumps does
    if isfield(d,'P'), P = d.P; elseif isfield(d,'P_diag'), P = diag(d.P_diag); else P = eye(15); end
    if isfield(d,'R'), R = d.R; elseif isfield(d,'R_diag'), R = diag(d.R_diag); else R = []; end
    sensor = 'mag';
    if isfield(d,'sensor'), sensor = d.sensor; end

    H = zeros(max(1,numel(z_clean)), 15);
    % try default mag H based on h if available
    if isfield(d,'h') && ~isempty(d.h)
        try
            H = [zeros(3,6), quat_lib('skew', d.h(:)), zeros(3,6)];
        catch
            H = zeros(numel(z_clean), 15);
        end
    end

    divergence_guard = DivergenceGuard(struct());
    noiseEstimator = NoiseEstimator(10);
    ctx = struct(); if isfield(d,'k'), ctx.k = d.k; end; ctx.z = z_clean; ctx.h = d.h; if exist('P','var'), ctx.P_diag = diag(P); end; if exist('R','var') && ~isempty(R), ctx.R_diag = diag(R); end

    try
        [should_update_orig, y_used_orig, K_used_orig, dx_used_orig, diag_orig] = OutlierGuard.checkAndApply(sensor, z_orig, d.h, H, P, R, [], [], divergence_guard, noiseEstimator, ctx);
    catch e
        fprintf('Original OutlierGuard error: %s\n', e.message);
        diag_orig = struct(); should_update_orig = false; end
    try
        [should_update_new, y_used_new, K_used_new, dx_used_new, diag_new] = OutlierGuard.checkAndApply(sensor, z_clean, d.h, H, P, R, [], [], divergence_guard, noiseEstimator, ctx);
    catch e
        fprintf('Preprocessed OutlierGuard error: %s\n', e.message);
        diag_new = struct(); should_update_new = false; end

    % print comparison
    smin_o = NaN; d2_o = NaN;
    if exist('diag_orig','var') && isstruct(diag_orig)
        if isfield(diag_orig,'S_minR_added'), smin_o = diag_orig.S_minR_added; end
        if isfield(diag_orig,'mahal_d2'), d2_o = diag_orig.mahal_d2; end
    end
    smin_n = NaN; d2_n = NaN;
    if exist('diag_new','var') && isstruct(diag_new)
        if isfield(diag_new,'S_minR_added'), smin_n = diag_new.S_minR_added; end
        if isfield(diag_new,'mahal_d2'), d2_n = diag_new.mahal_d2; end
    end
    fprintf('orig should_update=%d, S_minR_added=%g, mahal_d2=%g\n', double(should_update_orig), smin_o, d2_o);
    fprintf('prep should_update=%d, S_minR_added=%g, mahal_d2=%g\n', double(should_update_new), smin_n, d2_n);
end
end
