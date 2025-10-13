function [q, P] = update_mag(q, P, m_meas)
    % UPDATE_MAG  地磁気更新（yaw 補正） using adaptive R estimation

    % world magnetic vector (approx north pointing)
    m_world = [0; 50; 0];

    % predicted magnetic vector in body frame (from predicted attitude q)
    Rb = quat_lib('quat_to_rotm', q);
    h_mag = Rb' * m_world;

    q_obs = quat_lib('vector_to_quat', h_mag, m_meas);

    % Convert observation quaternion to a small-angle vector (3x1)
    qw = q_obs(1);
    qv = q_obs(2:4);
    if abs(qw) < 1e-8
        % fallback to small-angle approx if scalar part too small
        dtheta_obs = 2 * qv;
    else
        % general conversion for small-to-moderate angles
        dtheta_obs = 2 * qv / qw;
    end

    % measurement matrix: direct observation of attitude error (orientation states 7:9)
    H = [zeros(3,6), eye(3), zeros(3,6)];

    % use the small-angle vector as the measurement
    z_theta = dtheta_obs;

    % initial R guess for orientation measurement (tunable)
    R0 = eye(3) * 0.1;

    % call adaptive R estimator using the orientation innovation
    y0 = z_theta; % predicted measurement is zero
    params = adaptive_R_update(struct(), y0, H, P, R0, {struct('name','mag3','range',1:3)});
    if isfield(params,'kf') && isfield(params.kf,'R_est') && isfield(params.kf.R_est,'mag3')
        R_est = diag(params.kf.R_est.mag3);
    else
        R_est = R0;
    end

    % compute innovation; predicted measurement h_theta = zeros(3,1)
    [y, S, R_used] = compute_innovation_and_S(z_theta, zeros(3,1), H, P, R_est, struct());
    K = compute_kalman_gain(P, H, S);

    dx = K * y;

    % apply full small-angle correction (on 3 orientation states)
    dtheta = dx(7:9);
    dq = quat_lib('small_angle_quat', dtheta);
    q = quat_lib('quatmultiply', q, dq);
    q = quat_lib('quatnormalize', q);

    x_pred = zeros(15,1);
    [~, P] = update_state_covariance(x_pred, P, K, H, y, R_used);
end