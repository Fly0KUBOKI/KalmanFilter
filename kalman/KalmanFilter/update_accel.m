function [p, v, q, ba, bg, P] = update_accel(p, v, q, ba, bg, P, a_meas, dt)
    % UPDATE_ACCEL  加速度センサによる roll/pitch 補正（静止時のみ）
    % This function estimates measurement noise R from innovations using adaptive_R_update

    g_world = [0;0;9.81];
    Rb = quat_lib('quat_to_rotm', q);

    % Only use accel gravity update when vehicle is approximately stationary.
    % Prevents centripetal/linear accelerations from being absorbed into accel bias.
    accel_norm = norm(a_meas);
    stationary_thresh = 0.5; % m/s^2 tolerance around gravity magnitude
    if abs(accel_norm - 9.81) > stationary_thresh
        % skip accel-based update; return inputs unchanged
        return;
    end

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

    % apply small corrections (roll/pitch only) using Kalman correction dx
    % dx(7:9) contains small-angle attitude error estimate (theta_x, theta_y, theta_z)
    % For accel-based update only roll/pitch are observable; ignore yaw component.
    dtheta = [dx(7); dx(8); 0];
    

    % derive per-axis angle threshold from measurement noise R_used
    % Use Jacobian H_theta = -skew(h_accel) to map small-angle -> accel change
    H_theta = -quat_lib('skew', h_accel);
    user_min = 0.001; % radians (minimum threshold)
    apply_thresh_vec = zeros(3,1);
    for i = 1:3
        noise_var = R_used(i,i);
        meas_std_i = sqrt(max(noise_var, eps)); % m/s^2
        sens_i = max(norm(H_theta(:,i)), eps);
        theta_thresh_i = meas_std_i / sens_i;
        apply_thresh_vec(i) = max(user_min, theta_thresh_i);
        % zero out small corrections per-axis (compare absolute value)
        if abs(dtheta(i)) < apply_thresh_vec(i)
            dtheta(i) = 0;
        end
        bias_idx = 9 + i; % 10,11,12
        % use measurement std (meas_std_i) as a simple significance threshold
        if abs(dx(bias_idx)) > meas_std_i
            % ba(i) = ba(i) + dx(bias_idx);
        end
    end

    fprintf('dtheta: %f %f %f\n', dtheta(1), dtheta(2), dtheta(3));
    dq = quat_lib('small_angle_quat', dtheta);
    q = quat_lib('quatmultiply', q, dq);
    q = quat_lib('quatnormalize', q);
    
    % update covariance
    x_pred = zeros(15,1);
    x_pred(1:3) = p; x_pred(4:6) = v; x_pred(7:9) = zeros(3,1);
    x_pred(10:12) = ba; x_pred(13:15) = bg;
    [~, P] = update_state_covariance(x_pred, P, K, H, y, R_used);
    
end