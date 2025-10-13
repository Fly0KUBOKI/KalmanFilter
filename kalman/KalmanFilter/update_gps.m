function [p, v, q, ba, bg, P] = update_gps(p, v, q, ba, bg, P, lat, lon, alt, origin)
    % UPDATE_GPS  GPS の緯度経度高度をローカルメートルに変換して位置更新
    % origin = [lat0; lon0; alt0]
    lat0 = origin(1); lon0 = origin(2); alt0 = origin(3);

    % approximate conversion per prompt
    x = (lat - lat0) / (9.0e-6);
    y = (lon - lon0) / (9.0e-6 / cosd(lat0));
    z = alt - alt0;

    z_gps = [x; y; z];

    H = [eye(3), zeros(3,12)];

    z_meas = z_gps;
    h = p;

    % initial R guess (3x3)
    R0 = eye(3) * 5.0; % meters^2 nominal
    y0 = z_meas - h;

    params = adaptive_R_update(struct(), y0, H, P, R0, {struct('name','gps','range',1:3)});
    if isfield(params,'kf') && isfield(params.kf,'R_est') && isfield(params.kf.R_est,'gps')
        R_est = diag(params.kf.R_est.gps);
    else
        R_est = R0;
    end

    [yv, S, R_used] = compute_innovation_and_S(z_meas, h, H, P, R_est, struct());
    K = compute_kalman_gain(P, H, S);
    dx = K * yv;
    
    % % Apply corrections
    p = p + dx(1:3);
    v = v + dx(4:6);

    ba = ba + dx(10:12);
    % bg = bg + dx(13:15);

    x_pred = zeros(15,1);
    [~, P] = update_state_covariance(x_pred, P, K, H, yv, R_used);
end