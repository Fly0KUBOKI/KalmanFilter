function params = adaptive_R_update(params, y, H, P_pred, R, meas_tags)
% EMA-based adaptive measurement noise estimation per tag (keeps API similar to original)
if ~isfield(params,'kf') || ~isfield(params.kf,'ema_alpha')
    alpha = 0.1;
else
    alpha = params.kf.ema_alpha;
end
if ~isfield(params,'kf') || ~isfield(params.kf,'R_est')
    if ~isfield(params,'kf'), params.kf = struct(); end
    params.kf.R_est = struct();
end

% warmup configuration: number of samples to average before using EMA
if ~isfield(params.kf,'ema_warmup') || isempty(params.kf.ema_warmup)
    params.kf.ema_warmup = 100; % default: use first 100 samples mean
end
% containers to hold warmup sums and counts per tag
if ~isfield(params.kf,'R_warmup_count') || isempty(params.kf.R_warmup_count)
    params.kf.R_warmup_count = struct();
end
if ~isfield(params.kf,'R_warmup_sum') || isempty(params.kf.R_warmup_sum)
    params.kf.R_warmup_sum = struct();
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
    % guard against negative or NaN innovation estimates
    innov_comp(~isfinite(innov_comp)) = 0;
    % prevent overly negative innovations which would drive R_new negative
    innov_comp = max(innov_comp, 0);
    if ~isfield(params.kf.R_est, tag.name)
        params.kf.R_est.(tag.name) = R_diag(rng);
    end

    % initialize warmup containers for this tag if needed
    if ~isfield(params.kf.R_warmup_count, tag.name) || isempty(params.kf.R_warmup_count.(tag.name))
        params.kf.R_warmup_count.(tag.name) = 0;
        params.kf.R_warmup_sum.(tag.name) = zeros(size(innov_comp));
    end

    % warmup phase: accumulate innov_comp and use mean until enough samples
    count = params.kf.R_warmup_count.(tag.name);
    sumv = params.kf.R_warmup_sum.(tag.name);
    if count < params.kf.ema_warmup
        sumv = sumv + innov_comp;
        count = count + 1;
        params.kf.R_warmup_sum.(tag.name) = sumv;
        params.kf.R_warmup_count.(tag.name) = count;
        R_new = sumv / count; % use sample mean during warmup
        params.kf.R_est.(tag.name) = R_new;
    else
        % after warmup: run EMA
        R_prev = params.kf.R_est.(tag.name);
        if numel(R_prev) ~= numel(innov_comp)
            R_prev = repmat(mean(R_prev), numel(innov_comp), 1);
        end
        R_new = (1-alpha)*R_prev + alpha * innov_comp;
        params.kf.R_est.(tag.name) = R_new;
    end
    % clamp R within reasonable bounds to avoid collapse or explosion
    R_min = eps;
    R_max = 1e6;
    R_new(~isfinite(R_new)) = R_min;
    R_new = min(max(R_new, R_min), R_max);
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
