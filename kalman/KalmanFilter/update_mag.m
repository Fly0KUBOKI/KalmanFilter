function [q, P] = update_mag(q, P, m_meas)
    % UPDATE_MAG  地磁気更新（yaw 補正） using adaptive R estimation

    % world magnetic vector (approx north pointing)
    m_world = [0; 50; 0];

    % predicted magnetic vector in body frame (from predicted attitude q)
    Rb = quat_lib('quat_to_rotm', q);
    h_mag = Rb' * m_world;

    % --- New: use vector innovation (m_meas - h_mag) directly for a standard EKF ---
    % measurement z and predicted measurement h
    z = m_meas;
    h = h_mag;
    % fprintf('Measured mag: [%.2f, %.2f, %.2f], Predicted mag: [%.2f, %.2f, %.2f]\n', ...
    %     z(1), z(2), z(3), h(1), h(2), h(3));

    % measurement matrix: derivative of body-frame vector wrt small-angle orientation
    % Note: magnetic vector rotates opposite to the body rotation, so flip sign
    % compared to accel pattern. Use +skew(h) here to reflect that inverse rotation.
    H = [zeros(3,6), quat_lib('skew', h), zeros(3,6)];

    % initial R guess for magnetometer measurement (tunable)
    R0 = eye(3) * 0.1;

    % call adaptive R estimator using the vector innovation y0 = z - h
    y0 = z - h;
    params = adaptive_R_update(struct(), y0, H, P, R0, {struct('name','mag3','range',1:3)});
    if isfield(params,'kf') && isfield(params.kf,'R_est') && isfield(params.kf.R_est,'mag3')
        R_est = diag(params.kf.R_est.mag3);
    else
        R_est = R0;
    end

    % compute innovation and innovation covariance
    [y, S, R_used] = compute_innovation_and_S(z, h, H, P, R_est, struct());
    K = compute_kalman_gain(P, H, S);

    dx = K * y;

    % apply small-angle correction (orientation states 7:9)
    dtheta = dx(7:9);

    
    user_min = 0.001; % radians

    % Per-axis thresholding based on measurement std (sqrt of variance)
    apply_thresh = zeros(3,1);
    for i = 1:3
        noise = R_used(i,i)*0.1;
        meas_std_i = sqrt(max(noise, eps)); % units of measurement (nT)
        % map measurement std to angle threshold approximately using overall |h|
        theta_thresh_i = meas_std_i / max(norm(h), eps);
        apply_thresh(i) = max(user_min, theta_thresh_i);
        % zero out small corrections (compare absolute value)
        if abs(dtheta(i)) < apply_thresh(i)
            dtheta(i) = 0;
        end
    end
    dq = quat_lib('small_angle_quat', dtheta);
    q = quat_lib('quatmultiply', q, dq);
    q = quat_lib('quatnormalize', q);
   

    % update covariance (x_pred is not used for nominal here)
    x_pred = zeros(15,1);
    [~, P] = update_state_covariance(x_pred, P, K, H, y, R_used);
end