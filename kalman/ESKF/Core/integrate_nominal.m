function [p, v, q, ba, bg] = integrate_nominal(p, v, q, ba, bg, a_meas, w_meas, dt, g, gyro_noise_threshold, accel_noise_threshold)
% INTEGRATE_NOMINAL  ノミナル状態の数値積分（RK2, 台形則）

% バイアス補正
a = a_meas - ba;
w = w_meas - bg;

% 加速度の閾値処理
for i = 1:3
    if abs(a(i)) < accel_noise_threshold(i) * dt
        a(i) = 0;
    end
end

% 角速度の閾値処理（テスト用に無効化）
w_dt = w * dt;

% ジャイロ積分（クォータニオン更新）
w_dt_norm = norm(w_dt);
threshold = 1e-15;

if w_dt_norm > threshold
    half_angle = w_dt_norm / 2;
    if half_angle > 1e-6
        sin_half = sin(half_angle);
        cos_half = cos(half_angle);
        w_unit = w_dt / w_dt_norm;
        delta_q = [cos_half; w_unit * sin_half];
    else
        % 極小角度での安定的な近似（Taylor 展開）
        w_norm_sq = w_dt_norm * w_dt_norm;
        delta_q = [1.0 - w_norm_sq/8.0; w_dt * 0.5 * (1.0 - w_norm_sq/24.0)];
    end

    q = QuaternionLib.multiply(q, delta_q);

    % 四元数正規化
    q_norm = norm(q);
    if abs(q_norm - 1.0) > 1e-12
        q = q / q_norm;
    end
end

% 回転行列 body -> world
Rb = QuaternionLib.to_rotation_matrix(q);

% 世界座標系での比力
a_world = Rb * a;

% 速度・位置更新: Adams-Bashforth2 による2次精度積分
persistent prev_a_world prev_v prev_initialized
if isempty(prev_initialized)
    % 初回ステップ: forward Euler
    v_prev = v;
    v_candidate = v + (a_world + g) * dt;
    
    % 速度発散防止
    max_accel = 200.0;
    dv = v_candidate - v;
    dv_norm = norm(dv);
    max_dv = max_accel * dt;
    if dv_norm > max_dv
        dv = dv * (max_dv / dv_norm);
    end
    v = v + dv;
    
    % 速度クリップ
    max_velocity = 200;
    v = max(min(v, max_velocity), -max_velocity);

    % 位置更新
    p = p + v * dt + 0.5 * (a_world + g) * dt * dt;

    prev_a_world = a_world;
    prev_v = v_prev;
    prev_initialized = true;
else
    % AB2: v_{n+1} = v_n + dt*(3/2*a_n - 1/2*a_{n-1})
    a_prev = prev_a_world;
    v_old = v;
    v_new = v + dt * (1.5 * (a_world + g) - 0.5 * (a_prev + g));

    % 速度発散防止
    max_accel = 200.0;
    dv = v_new - v;
    dv_norm = norm(dv);
    max_dv = max_accel * dt;
    if dv_norm > max_dv
        dv = dv * (max_dv / dv_norm);
    end
    v = v + dv;

    % 速度クリップ
    max_velocity = 200;
    v = max(min(v, max_velocity), -max_velocity);

    % 位置更新
    p = p + dt * (1.5 * v_old - 0.5 * prev_v);

    prev_v = v_old;
    prev_a_world = a_world;
end

end