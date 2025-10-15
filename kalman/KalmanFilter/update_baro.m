function [p, P] = update_baro(p, P, pressure)
    % UPDATE_BARO  気圧を高度に変換し高度のみ補正
    P0 = 101325;
    alt_baro = 44330 * (1 - (pressure / P0)^0.1903);

    H = [0,0,1, zeros(1,12)];

    z = alt_baro;
    h = p(3);

    % initial R guess
    R0 = 1.0;
    y0 = z - h;

    params = adaptive_R_update(struct(), y0, H, P, R0, {struct('name','baro','range',1)});
    if isfield(params,'kf') && isfield(params.kf,'R_est') && isfield(params.kf.R_est,'baro')
        R_est = params.kf.R_est.baro;
    else
        R_est = R0;
    end

    [y, S, R_used] = compute_innovation_and_S(z, h, H, P, R_est, struct());
    K = compute_kalman_gain(P, H, S);

    dx = K * y;

    % apply significance threshold to altitude correction (meters)
    % derive measurement std from R_used and enforce a minimum threshold
    % so very small corrections are ignored
   
    
    meas_var = max(R_used, eps);
    meas_std = sqrt(meas_var);
    user_min_m = 0.1; % meters (minimum threshold to apply baro correction)
    apply_thresh = max(user_min_m, meas_std);

    % only apply vertical position correction if it exceeds threshold
    if abs(dx(3)) >= apply_thresh
        p(3) = p(3) + dx(3);
        fprintf('dz = %g m (thresh=%g)\n', dx(3), apply_thresh);
    end

    % prepare x_pred for covariance update (state after applying correction)
    x_pred = zeros(15,1);
    x_pred(1:3) = p;

    

    [~, P] = update_state_covariance(x_pred, P, K, H, y, R_used);
end