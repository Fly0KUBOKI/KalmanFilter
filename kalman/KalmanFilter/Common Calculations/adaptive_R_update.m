function params = adaptive_R_update(params, y, H, P_pred, R, meas_tags)
% EMA-based adaptive measurement noise estimation per tag.
% Inputs:
%  params - parameter struct (contains params.kf fields for state)
%  y, H, P_pred, R - innovation, measurement jacobian, predicted P and nominal R
%  meas_tags - cell array of structs with fields 'name' and 'range' indicating
%              which elements of y correspond to each measurement tag

% default alpha
if ~isfield(params,'kf') || ~isfield(params.kf,'ema_alpha')
    alpha = 0.05;
else
    alpha = params.kf.ema_alpha;
end

if ~isfield(params,'kf'), params.kf = struct(); end
if ~isfield(params.kf,'R_est')
    params.kf.R_est = struct();
end

% warmup configuration: number of samples to average before using EMA
if ~isfield(params.kf,'ema_warmup') || isempty(params.kf.ema_warmup)
    % default number of samples to collect before switching to EMA
    % shortened from 100 to 10 to reduce long initialization delays
    params.kf.ema_warmup = 0;
end
if ~isfield(params.kf,'R_warmup_count') || isempty(params.kf.R_warmup_count)
    params.kf.R_warmup_count = struct();
    params.kf.R_warmup_sum = struct();
end
        params.kf.ema_warmup = 0; % Set default ema_warmup to 0
% bounds for R estimates
if isfield(params.kf,'R_est_abs_max') && ~isempty(params.kf.R_est_abs_max)
    R_abs_max = max(eps, params.kf.R_est_abs_max);
else
    R_abs_max = 1e6;
end

HPHT = H * P_pred * H';
HPHT_diag = diag(HPHT);
R_diag = diag(R);

for i=1:numel(meas_tags)
    tag = meas_tags{i};
    rng = tag.range;
    if isempty(rng), continue; end

    res_sq = (y(rng)).^2;
    hpht_comp = HPHT_diag(rng);
    innov_comp = res_sq - hpht_comp;
    % guard against invalid values
    innov_comp(~isfinite(innov_comp)) = 0;
    innov_comp = max(innov_comp, 0);

    % ensure we have an initial R estimate for this tag
    if ~isfield(params.kf.R_est, tag.name)
        % if R_diag shorter than rng, expand sensibly
        if numel(R_diag) >= max(rng)
            params.kf.R_est.(tag.name) = R_diag(rng);
        else
            params.kf.R_est.(tag.name) = max(eps, mean(R_diag)) * ones(numel(rng),1);
        end
    end

    % initialize warmup containers
    if ~isfield(params.kf.R_warmup_count, tag.name) || isempty(params.kf.R_warmup_count.(tag.name))
        params.kf.R_warmup_count.(tag.name) = 0;
        params.kf.R_warmup_sum.(tag.name) = zeros(size(innov_comp));
    end

    count = params.kf.R_warmup_count.(tag.name);
    sumv = params.kf.R_warmup_sum.(tag.name);
    if count < params.kf.ema_warmup
        sumv = sumv + innov_comp;
        count = count + 1;
        params.kf.R_warmup_sum.(tag.name) = sumv;
        params.kf.R_warmup_count.(tag.name) = count;
        R_new = sumv / count;
        params.kf.R_est.(tag.name) = R_new;
    else
        R_prev = params.kf.R_est.(tag.name);
        if numel(R_prev) ~= numel(innov_comp)
            R_prev = repmat(mean(R_prev), numel(innov_comp), 1);
        end
        R_new = (1-alpha)*R_prev + alpha * innov_comp;
        % optional per-step growth cap
        if isfield(params.kf,'R_est_max_mult') && ~isempty(params.kf.R_est_max_mult)
            mult = max(1, params.kf.R_est_max_mult);
            R_prev_safe = max(R_prev, eps);
            R_new = min(R_new, R_prev_safe * mult);
        end
        params.kf.R_est.(tag.name) = R_new;
    end

    % clamp R_new
    R_new(~isfinite(R_new)) = eps;
    R_new = min(max(R_new, eps), R_abs_max);
    params.kf.R_est.(tag.name) = R_new;

    % write back into params.noise in expected format
    switch tag.name
        case 'gps'
            params.noise.gps = sqrt(R_new(:));
        case 'vel'
            params.noise.vel = sqrt(R_new(:));
        case 'accel3'
            params.noise.accel3 = sqrt(R_new(:));
        case 'gyro3'
            val = sqrt(mean(R_new));
            params.noise.gyro3 = repmat(val,1,3);
        case 'mag3'
            params.noise.mag3 = sqrt(R_new(:));
        case 'baro'
            params.noise.baro = sqrt(mean(R_new));
        case 'heading'
            params.noise.heading = sqrt(mean(R_new));
        otherwise
            params.noise.(tag.name) = sqrt(R_new(:));
    end
end
end
