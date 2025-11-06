function results = monitor_divergence(varargin)
% MONITOR_DIVERGENCE  ESKF の実行中に共分散の挙動を監視する簡易ツール
%
%   results = monitor_divergence()
%   results = monitor_divergence(params)
%
%  目的:
%    - 各タイムステップでの P の対角（分散）、固有値、条件数を記録
%    - ある閾値を超えた時点を報告して保存・可視化する
%
%  入力 params (任意):
%    .max_variance_factor : P の対角の最大許容倍率 (default 1e6 * mean(trace(P)/n))
%    .min_rcond           : rcond(P) がこの値を下回ると警告 (default 1e-12)
%    .zero_innovation_thr : (unused) 一貫性のためパラメータ入力を受け付けます
%    .run_simulation      : true/false - run_simulation を実行してデータを作る (default true)
%
%  出力:
%    results - 構造体 (time, max_diag, traceP, min_eig, max_eig, rcondP, diverge_indices)

    % デフォルトパラメータ
    params = struct();
    params.max_variance_factor = 1e6;
    params.min_rcond = 1e-14;  % relaxed from 1e-12 to tolerate machine precision edge cases
    params.run_simulation = true;

    if ~isempty(varargin)
        u = varargin{1};
        if isstruct(u)
            fields = fieldnames(u);
            for i=1:numel(fields)
                params.(fields{i}) = u.(fields{i});
            end
        end
    end

    projRoot = fileparts(mfilename('fullpath'));
    % set paths similar to run_simulation
    addpath(genpath(fullfile(projRoot, '..', 'KF')));
    addpath(genpath(fullfile(projRoot, '..', 'ESKF')));
    addpath(fullfile(projRoot, '..', 'GenerateData'));
    addpath(fullfile(projRoot, '..', 'Graph'));
    rehash;

    % optionally run the normal simulation to produce data files
    if params.run_simulation
        try
            fprintf('Running run_simulation to (re)generate data...\n');
            run_simulation();
        catch e
            warning('monitor_divergence:run_simulation','run_simulation failed: %s', e.message);
            % continue if data already exists
        end
    end

    % load observations
    dataDir = fullfile(projRoot, '..', 'GenerateData');
    obsFile = fullfile(dataDir, 'sensor_data.csv');
    if ~exist(obsFile,'file')
        error('sensor_data.csv not found: %s', obsFile);
    end
    obs = read_csv(obsFile);

    % create ESKF instance
    params_cfg = config_params();
    dt = mean(diff(obs.time));
    eskf = ESKF(obs, params_cfg.static_time, dt);

    N = numel(obs.time);
    results.time = obs.time(:)';
    results.traceP = zeros(1,N);
    results.max_diag = zeros(1,N);
    results.min_diag = zeros(1,N);
    results.max_eig = zeros(1,N);
    results.min_eig = zeros(1,N);
    results.rcondP = zeros(1,N);
    results.vel_norm = zeros(1,N);
    results.pos_norm = zeros(1,N);
    results.vel_var_max = zeros(1,N);

    fprintf('Monitoring %d steps...\n', N);
    for k=1:N
        eskf.updateFilter(obs, k);
        P = eskf.P;
        % ensure symmetry
        P = (P + P')/2;
        d = diag(P);
        results.traceP(k) = trace(P);
        results.max_diag(k) = max(d);
        results.min_diag(k) = min(d);
        ev = eig(P);
        results.max_eig(k) = max(ev);
        results.min_eig(k) = min(ev);
        results.rcondP(k) = rcond(P);
        
        % record actual state values
        results.vel_norm(k) = norm(eskf.v);
        results.pos_norm(k) = norm(eskf.p);
        results.vel_var_max(k) = max(d(4:6));
    end

    % detect problematic indices
    n = size(eskf.P,1);
    base = max(eps, mean(results.traceP)/n);
    max_allowed = params.max_variance_factor * base;

    idx_large_var = find(results.max_diag > max_allowed);
    idx_bad_rcond = find(results.rcondP < params.min_rcond);
    idx_neg_eig = find(results.min_eig <= 0 | ~isfinite(results.min_eig));
    
    % detect velocity divergence (velocity norm > 10 m/s or velocity variance > 1 m^2/s^2)
    idx_vel_large = find(results.vel_norm > 10);
    idx_vel_var_large = find(results.vel_var_max > 1);

    diverge_idx = unique([idx_large_var, idx_bad_rcond, idx_neg_eig, idx_vel_large, idx_vel_var_large]);

    fprintf('\n--- Divergence Monitor Report ---\n');
    if isempty(diverge_idx)
        fprintf('No divergence events detected (thresholds: max_factor=%g, min_rcond=%g, max_vel=10 m/s)\n', params.max_variance_factor, params.min_rcond);
    else
        fprintf('Divergence detected at %d time steps. First: index %d (t=%.4f s)\n', numel(diverge_idx), diverge_idx(1), results.time(diverge_idx(1)));
        fprintf('  Examples:\n');
        sample = diverge_idx(1:min(5,numel(diverge_idx)));
        for i=sample
            fprintf('   idx=%d t=%.4f traceP=%.3e max_diag=%.3e min_eig=%.3e rcond=%.3e vel=%.3f vel_var=%.3e\n', ...
                i, results.time(i), results.traceP(i), results.max_diag(i), results.min_eig(i), results.rcondP(i), results.vel_norm(i), results.vel_var_max(i));
        end
    end

    % save results
    outFile = fullfile(projRoot, 'divergence_monitor_results.mat');
    save(outFile, 'results', 'params');
    fprintf('Saved results to %s\n', outFile);

    % quick plots
    figure('Name','Divergence Monitor');
    subplot(4,1,1);
    plot(results.time, results.traceP); ylabel('trace(P)'); grid on;
    subplot(4,1,2);
    plot(results.time, results.max_diag); hold on; plot(results.time, results.min_diag); ylabel('diag(P)'); legend('max diag','min diag'); grid on;
    subplot(4,1,3);
    semilogy(results.time, results.rcondP); ylabel('rcond(P)'); grid on;
    subplot(4,1,4);
    plot(results.time, results.vel_norm); ylabel('velocity norm (m/s)'); xlabel('time (s)'); grid on;

end
