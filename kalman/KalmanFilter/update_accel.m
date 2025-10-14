function [p, v, q, ba, bg, P] = update_accel(p, v, q, ba, bg, P, a_meas, dt)
    % UPDATE_ACCEL  加速度センサによる roll/pitch 補正（静止時のみ）
    % This function estimates measurement noise R from innovations using adaptive_R_update

    g_world = [0;0;9.81];
    Rb = quat_lib('quat_to_rotm', q);

    % predicted specific force in body frame (model)
    h_accel = Rb' * g_world;

    % measurement (bias removed)
    z = a_meas - ba;

    % observation matrix (3 x 15)
    H = [zeros(3,3), zeros(3,3), -quat_lib('skew', h_accel), -eye(3), zeros(3,3)];

    % initial R guess
    R0 = eye(3) * (0.01 * sqrt(max(dt,eps)));

    % compute initial innovation
    y0 = z - h_accel;

    % call adaptive R estimator: adaptive_R_update expects params, y, H, P_pred, R, meas_tags
    params = struct();
    params = adaptive_R_update(params, y0, H, P, R0, {struct('name','accel3','range',1:3)});
    % retrieve estimated R for accel
    if isfield(params,'kf') && isfield(params.kf,'R_est') && isfield(params.kf.R_est,'accel3')
        R_est = diag(params.kf.R_est.accel3);
    else
        R_est = R0;
    end

    % compute innovation and S (use compute_innovation_and_S for reg/gating)
    [y, S, R_used] = compute_innovation_and_S(z, h_accel, H, P, R_est, struct());

    % Kalman gain and update
    K = compute_kalman_gain(P, H, S);
    dx = K * y;

    % apply small corrections (roll/pitch only)
    % dtheta_rp = [dx(7); dx(8); 0];
    % dq = quat_lib('small_angle_quat', dtheta_rp);
    % q = quat_lib('quatmultiply', q, dq);
    % q = quat_lib('quatnormalize', q);

    % update accel bias
    ba = ba + dx(10:12);
    % fprintf('ba: [%f, %f, %f]\n', ba(1), ba(2), ba(3));

    % update covariance
    x_pred = zeros(15,1);
    x_pred(1:3) = p; x_pred(4:6) = v; x_pred(7:9) = zeros(3,1);
    x_pred(10:12) = ba; x_pred(13:15) = bg;
    [~, P] = update_state_covariance(x_pred, P, K, H, y, R_used);
    
end