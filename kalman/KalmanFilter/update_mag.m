function [q, P] = update_mag(q, P, m_meas)
    % UPDATE_MAG  地磁気更新（yaw 補正） using adaptive R estimation

    % world magnetic vector (approx north pointing)
    m_world = [50; 0; 0];

    Rb = quat_lib('quat_to_rotm', q);
    h_mag = Rb' * m_world;

    H = [zeros(3,3), zeros(3,3), -quat_lib('skew', h_mag), zeros(3,3), zeros(3,3)];

    z = m_meas;

    % initial R guess
    R0 = eye(3) * 1.0; % nominal mag noise (tunable)
    % compute innovation
    y0 = z - h_mag;

    % estimate R adaptively for tag 'mag3'
    params = adaptive_R_update(struct(), y0, H, P, R0, {struct('name','mag3','range',1:3)});
    if isfield(params,'kf') && isfield(params.kf,'R_est') && isfield(params.kf.R_est,'mag3')
        R_est = diag(params.kf.R_est.mag3);
    else
        R_est = R0;
    end

    [y, S, R_used] = compute_innovation_and_S(z, h_mag, H, P, R_est, struct());
    K = compute_kalman_gain(P, H, S);

    dx = K * y;
    % apply yaw-only correction
    dtheta_yaw = [0; 0; dx(9)];
    dq = quat_lib('small_angle_quat', dtheta_yaw);
    q = quat_lib('quatmultiply', q, dq);
    q = quat_lib('quatnormalize', q);

    x_pred = zeros(15,1);
    [~, P] = update_state_covariance(x_pred, P, K, H, y, R_used);
end