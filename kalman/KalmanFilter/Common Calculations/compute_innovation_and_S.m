function [y, S, R_out] = compute_innovation_and_S(z, h, H, P_pred, R, params)
% Compute innovation y and innovation covariance S.
% Handles NaN/Inf in z/h by inflating R diag and regularizes S.

y = z - h;

% initial S
S = H * P_pred * H' + R;

% protect against NaN/Inf in innovation
nan_idx = isnan(y) | isinf(y);
if any(nan_idx)
    y(nan_idx) = 0;
    rd = diag(S);
    large = max(1e6, 1e3 * median(rd(rd>0)));
    % get R diagonal (handle scalar R)
    if isempty(R)
        R_diag = zeros(numel(y),1);
    else
        R_diag = diag(R);
    end
    % ensure length match: if R provides fewer entries than y, expand using
    % median of positive entries or a fallback based on S diagonal
    if numel(R_diag) < numel(y)
        pos = R_diag(R_diag>0);
        if ~isempty(pos)
            fill = median(pos);
        else
            sd = diag(H*P_pred*H');
            fill = max(median(sd(sd>0)), eps);
        end
        R_diag = [R_diag; repmat(fill, numel(y)-numel(R_diag), 1)];
    end
    R_diag(~isfinite(R_diag) | R_diag<=0) = eps;
    R_diag(nan_idx) = large;
    R = diag(R_diag);
    S = H * P_pred * H' + R;
end

% ensure R diagonal positive finite
R_diag = diag(R);
R_diag(~isfinite(R_diag) | R_diag <= 0) = eps;
R = diag(R_diag);
S = H * P_pred * H' + R;

% regularize S if ill-conditioned
n = size(S,1);
if n > 0
    r = rcond(S);
    reg_scale = 1e-8; iter = 0;
    base = max(eps, trace(S)/n);
    while (isnan(r) || r < 1e-12) && iter < 8
        reg = (10^iter) * reg_scale * base;
        S = S + reg * eye(n);
        r = rcond(S);
        iter = iter + 1;
    end
    if isnan(r) || r < 1e-12
        warning('compute_innovation_and_S:IllConditionedS','S is ill-conditioned. rcond=%g', r);
    end
    if isfield(params,'kf') && isfield(params.kf,'debug') && params.kf.debug
        fprintf('compute_innovation_and_S: rcond after reg = %g (iter=%d)\n', r, iter);
    end
end

R_out = R;
end
