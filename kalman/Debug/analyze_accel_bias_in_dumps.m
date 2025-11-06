function analyze_accel_bias_in_dumps()
% ANALYZE_ACCEL_BIAS_IN_DUMPS  Scan divergence dumps and report large accel-bias updates
%  Checks dx (computed from K_approx*y if necessary) and reports norm of bias components (idx 10:12)

cd(fileparts(mfilename('fullpath')));
base = fileparts(pwd); results = fullfile(base,'Results');

% search recursively under Results for any divergence dump files
files = [dir(fullfile(results,'**','divergence_full_dump_*.mat')); dir(fullfile(results,'**','divergence_dump_*.mat')); dir(fullfile(results,'**','divergence_dump_*')); dir(fullfile(results,'**','divergence_full_dump_*'))];
if isempty(files)
    fprintf('No divergence dumps found in %s\n', results); return;
end

thresh_high = 0.05; % m/s^2 - large bias update
thresh_warn = 0.01; % m/s^2 - warning

report = struct('file',{},'k',{},'sensor',{},'bias_norm',{},'K_bias_norm',{},'P_bias_mean',{},'mahal_d2',{});
count_high = 0; count_warn = 0;

for i=1:length(files)
    fn = fullfile(files(i).folder, files(i).name);
    try
        S = load(fn);
    catch
        continue;
    end
    if isfield(S,'dump'), d = S.dump; else f=fieldnames(S); d=S.(f{1}); end
    sensor = 'unknown'; if isfield(d,'sensor'), sensor = lower(d.sensor); end
    % get y
    y = [];
    if isfield(d,'y'), y = d.y(:); elseif isfield(d,'z') && isfield(d,'h'), y = d.z(:)-d.h(:); end
    % get P
    if isfield(d,'P'), P = d.P; elseif isfield(d,'P_diag'), P = diag(d.P_diag); else P = []; end
    % get K_approx
    K = [];
    if isfield(d,'K_approx'), K = d.K_approx; end
    % get dx if present
    dx = [];
    if isfield(d,'dx_used')
        dx = d.dx_used;
    elseif isfield(d,'dx')
        dx = d.dx;
    elseif ~isempty(K) && ~isempty(y)
        % try compute dx = K * y
        try
            dx = K * y;
        catch
            dx = [];
        end
    end

    bias_norm = NaN; K_bias_norm = NaN; P_bias_mean = NaN; d2 = NaN;
    if ~isempty(dx) && numel(dx) >= 12
        bd = dx(10:12);
        bias_norm = norm(bd);
    end
    if ~isempty(K) && size(K,1) >= 12
        try
            Kb = K(10:12,:);
            K_bias_norm = norm(Kb,'fro');
        catch
        end
    end
    if ~isempty(P) && numel(P) >= 12
        P_bias_mean = mean(diag(P(10:12,10:12)));
    end
    if isfield(d,'mahal_d2'), d2 = d.mahal_d2; end

    if ~isnan(bias_norm) && bias_norm >= thresh_warn
        rec.file = files(i).name;
        rec.k = getfield(d,'k',NaN);
        rec.sensor = sensor;
        rec.bias_norm = bias_norm;
        rec.K_bias_norm = K_bias_norm;
        rec.P_bias_mean = P_bias_mean;
        rec.mahal_d2 = d2;
        report(end+1) = rec; %#ok<AGROW>
        if bias_norm >= thresh_high, count_high = count_high + 1; else count_warn = count_warn + 1; end
    end
end

% sort descending by bias_norm
if ~isempty(report)
    biases = [report.bias_norm]; [~, idx] = sort(biases,'descend'); report = report(idx);
end

fprintf('\nAccel bias update analysis:\n');
fprintf('Total dumps scanned: %d\n', length(files));
fprintf('Warnings (>%g): %d, High(>%g): %d\n', thresh_warn, count_warn, thresh_high, count_high);

if ~isempty(report)
    fprintf('\nTop offending dumps (bias_norm, file, k, sensor, K_bias_norm, P_bias_mean, mahal_d2):\n');
    for i=1:min(30,length(report))
        r = report(i);
        fprintf(' %6.4f, %s, k=%g, sensor=%s, Kb_norm=%.4g, P_bias=%.4g, d2=%.4g\n', r.bias_norm, r.file, r.k, r.sensor, r.K_bias_norm, r.P_bias_mean, r.mahal_d2);
    end
else
    fprintf('No dumps exceeded the warning threshold (%.4g)\n', thresh_warn);
end

% Save report to Results for later inspection
outfn = fullfile(results, sprintf('accel_bias_report_%s.mat', datestr(now,'yyyymmdd_HHMMSS')));
try
    save(outfn,'report','thresh_warn','thresh_high');
    fprintf('\nSaved report to %s\n', outfn);
catch
    fprintf('\nCould not save report to %s\n', outfn);
end
end
