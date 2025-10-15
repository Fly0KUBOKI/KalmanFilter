function [p, v, q, ba, bg, P] = update_gps(p, v, q, ba, bg, P, lat, lon, alt, origin, gps_dt)
    % UPDATE_GPS  GPS の緯度経度高度をローカルメートルに変換して位置更新
    % origin = [lat0; lon0; alt0]
    lat0 = origin(1); lon0 = origin(2); alt0 = origin(3);

    % approximate conversion per prompt
    x = (lat - lat0) / (9.0e-6);
    y = (lon - lon0) / (9.0e-6 / cosd(lat0));
    z = alt - alt0;

    z_gps = [x; y; z];
    v_gps = zeros(3,1);

    % Optionally form a velocity observation from the previous GPS sample.
    % Use persistent previous GPS and finite-difference, and convert to m/s by dividing by gps_dt.
    persistent prev_z_gps;
    if ~isempty(prev_z_gps) && nargin >= 10 && ~isempty(gps_dt) && gps_dt > 0
        v_gps = (z_gps - prev_z_gps) / gps_dt; % meters per second
    elseif ~isempty(prev_z_gps)
        % fallback: provide delta-per-sample (legacy behavior)
        v_gps = z_gps - prev_z_gps;
    end
    prev_z_gps = z_gps;

    % measurement: [position; velocity_delta]
    H = [eye(3), zeros(3,12);
            zeros(3,3), eye(3), zeros(3,9)];
    z_meas = [z_gps; v_gps];
    h = [p; v];
    % fprintf('P[%f %f %f], meas[%f %f %f]\n', p(1), p(2), p(3), z_meas(1), z_meas(2), z_meas(3));

    % initial R guess (6x6) - keep reasonable defaults, minimal change
    R0 = diag([5.0,5.0,5.0, 5.0,5.0,5.0]);
    y0 = z_meas - h;

    params = adaptive_R_update(struct(), y0, H, P, R0, {struct('name','gps','range',1:6)});

    if isfield(params,'kf') && isfield(params.kf,'R_est') && isfield(params.kf.R_est,'gps')
        R_est = diag(params.kf.R_est.gps);
    else
        R_est = R0;
    end

    [yv, S, R_used] = compute_innovation_and_S(z_meas, h, H, P, R_est, struct());
    K = compute_kalman_gain(P, H, S);
    dx = K * yv;

    
    vel_var = diag(R_used(4:6,4:6));
    meas_std = sqrt(max(vel_var, eps));
    % threshold: use meas_std as basic threshold, and a small user_min
    user_min = 0.001; % meters per GPS-sample (small floor)
    thresh = zeros(3,1);
    for i = 1:3
        thresh(i) = max(user_min, meas_std(i));
        if abs(dx(3 + i)) < thresh(i)*0.01
            dx(3 + i) = 0;
        end
    end
    % % Apply corrections
    p = p + dx(1:3);
    v = v + dx(4:6);
    % fprintf('P[%.2f, %.2f], V[%.2f, %.2f]\n', p(1), p(2), v(1), v(2));


    % absorbing unmodelled motion into sensor biases.
    ba = ba + dx(10:12);
    % bg = bg + dx(13:15);

    x_pred = zeros(15,1);
    [~, P] = update_state_covariance(x_pred, P, K, H, yv, R_used);
end