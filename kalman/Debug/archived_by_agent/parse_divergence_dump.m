function parse_divergence_dump(fn)
% PARSE_DIVERGENCE_DUMP  divergence dump を読みやすく表示する簡易ユーティリティ
%   parse_divergence_dump(fn)
%
%   引数:
%     fn - ダンプファイルのパス（省略可: 最新の divergence_dump_*.mat を使用）
%
%   出力: ダンプの主要フィールドをコマンドウィンドウに表示します。

if nargin < 1 || isempty(fn)
    ddir = fullfile(fileparts(mfilename('fullpath')), '..', 'Results');
    files = dir(fullfile(ddir, 'divergence_dump_*.mat'));
    if isempty(files)
        files = dir(fullfile(ddir, 'divergence_full_dump_*.mat'));
    end
    if isempty(files)
        error('No divergence dump files found in %s', ddir);
    end
    % newest
    [~, idx] = max([files.datenum]);
    fn = fullfile(files(idx).folder, files(idx).name);
end

fprintf('Loading dump: %s\n', fn);
S = load(fn);
if isfield(S,'dump')
    d = S.dump;
else
    % maybe saved differently
    f = fieldnames(S);
    d = S.(f{1});
end

disp('--- dump keys ---'); disp(fieldnames(d));
if isfield(d,'k'), fprintf('k = %s\n', mat2str(d.k)); end
if isfield(d,'sensor'), fprintf('sensor = %s\n', d.sensor); end
if isfield(d,'time'), fprintf('time = %s\n', d.time); end

if isfield(d,'z'), fprintf('\nz (measurement):\n'); disp(d.z(:)'); end
if isfield(d,'h'), fprintf('\nh (predicted meas):\n'); disp(d.h(:)'); end

if isfield(d,'P')
    diagP = diag(d.P);
    fprintf('\nP diag (first 15 elements or all):\n');
    disp(diagP(:)');
end

if isfield(d,'R')
    try
        diagR = diag(d.R);
        fprintf('\nR diag:\n'); disp(diagR(:)');
    catch
        fprintf('\nR: (non-square?)\n'); disp(d.R);
    end
end

% Innovation covariance / S
if isfield(d,'S')
    try
        diagS = diag(d.S);
        fprintf('\nS diag:\n'); disp(diagS(:)');
        try
            r = rcond(d.S);
            fprintf('S rcond = %.3e\n', r);
        catch
        end
        % Mahalanobis d^2
        if isfield(d,'y')
            try
                invS = pinv(d.S);
                d2 = real(d.y(:)' * (invS * d.y(:)));
                fprintf('Mahalanobis d^2 = %.6g\n', d2);
            catch
            end
        end
    catch
    end
elseif isfield(d,'S_rcond')
    fprintf('\nS_rcond = %.3e\n', d.S_rcond);
end

% Extra diagnostics and quick recommendations
try
    if isfield(d,'S')
        S_r = NaN; try S_r = rcond(d.S); catch, end
        if isfield(d,'S_regularized') && d.S_regularized
            fprintf('Note: S was regularized in the filter. jitter=%.3e, rcond_after=%.3e\n', getfield(d,'S_jitter',NaN), getfield(d,'S_rcond_after',NaN));
        end
        % chi2 threshold suggestion for sensor dim
        if isfield(d,'y')
            sensor_dim = length(d.y(:));
            try
                chi2_thr = chi2inv(0.975, sensor_dim);
            catch
                chi2_thr = (3.0^2) * sensor_dim;
            end
            if exist('d2','var') && ~isempty(d2)
                if d2 > chi2_thr * 25
                    fprintf('Recommendation: Mahalanobis extremely large (d2 >> thr); consider skipping this update or increasing outlier skip multiplier.\n');
                elseif d2 > chi2_thr
                    fprintf('Recommendation: Mahalanobis moderately large (d2 > chi2_thr); current code attenuates innovation. Consider tightening gating or lowering gain.\n');
                else
                    fprintf('Mahalanobis within chi2 threshold.\n');
                end
            end
        end
        if exist('S_r','var') && isfinite(S_r) && S_r < 1e-12
            fprintf('Warning: S is ill-conditioned (rcond < 1e-12). Consider increasing regularization or jitter.\n');
        end
    elseif isfield(d,'S_rcond')
        if d.S_rcond < 1e-12
            fprintf('Warning: S_rcond is low (<1e-12). Consider regularization.\n');
        end
    end
catch
end

if isfield(d,'K_approx')
    try
        fprintf('\nK_approx norm (Frobenius) = %.6g\n', norm(d.K_approx,'fro'));
    catch
    end
end

% show any raw_window if present
if isfield(d,'raw_window')
    fprintf('\nRaw sensor window rows: %d x %d\n', size(d.raw_window,1), size(d.raw_window,2));
    fprintf('First row:\n'); disp(d.raw_window(1,:));
    fprintf('Last row:\n'); disp(d.raw_window(end,:));
end

fprintf('\nDone.\n');
end
