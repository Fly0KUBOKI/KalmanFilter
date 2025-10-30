function [p, v, q, ba, bg, P] = update_accel(p, v, q, ba, bg, P, a_meas, dt)
    % UPDATE_ACCEL  加速度センサによる roll/pitch 補正（静止時のみ）
    % This function estimates measurement noise R from innovations using adaptive_R_update

    persistent count accel_int dt_sum
    if isempty(count)
        count = 0;
        accel_int = zeros(3,1);
        dt_sum = 0;
    end
    
    count = count + 1;
    accel_int = accel_int + a_meas * dt;
    dt_sum = dt_sum + dt;
    
    if count < 4
        return;
    end
    
    a_meas = accel_int / dt_sum;
    dt = dt_sum;
    count = 0;
    accel_int = zeros(3,1);
    dt_sum = 0;

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

    % Convert body-frame acceleration to world frame
    % 機体座標系の加速度測定値を世界座標系に変換
    a_body_corrected = a_meas - ba; % バイアス補正
    a_world = Rb * a_body_corrected; % 世界座標系に変換
    
    % predicted specific force in world frame (model)
    % 世界座標系での予測加速度（重力のみ）
    h_accel = g_world;

    % measurement in world frame
    z = a_world;

    % observation matrix (3 x 15)
    % H_theta: da/dtheta in world frame = -R_b * skew(a_body_corrected)
    H_theta = -Rb * quat_lib('skew', a_body_corrected);
    H = [zeros(3,3), zeros(3,3), H_theta, -Rb, zeros(3,3)];

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
    % Use Jacobian H_theta to map small-angle -> accel change in world frame
    user_min = 0.001; % radians (minimum threshold)
    apply_thresh_vec = zeros(3,1);
    for i = 1:3
        noise_var = R_used(i,i);
        meas_std_i = sqrt(max(noise_var, eps)); % m/s^2
        sens_i = max(norm(H_theta(:,i)), eps);
        theta_thresh_i = meas_std_i / sens_i;
        apply_thresh_vec(i) = max(user_min, theta_thresh_i);
        % zero out small corrections per-axis (compare absolute value)
        if abs(dtheta(i)) < apply_thresh_vec(i)*2
            dtheta(i) = 0;
        end
        bias_idx = 9 + i; % 10,11,12
        % use measurement std (meas_std_i) as a simple significance threshold
        if abs(dx(bias_idx)) < meas_std_i*2
            dx(bias_idx) = 0;
        end
    end

    % fprintf('dtheta applied: [%.4f, %.4f, %.4f] \n', ...
    %     dtheta(1), dtheta(2), dtheta(3));
    % dq = quat_lib('small_angle_quat', dtheta);
    % q = quat_lib('quatmultiply', q, dq);
    % q = quat_lib('quatnormalize', q);
    
    % バイアス更新: dx(10:12)は世界座標系なので機体座標系に変換
    ba = ba + Rb' * dx(10:12);
    
    % update covariance
    x_pred = zeros(15,1);
    x_pred(1:3) = p; x_pred(4:6) = v; x_pred(7:9) = zeros(3,1);
    x_pred(10:12) = ba; x_pred(13:15) = bg;
    [~, P] = update_state_covariance(x_pred, P, K, H, y, R_used);
    
end