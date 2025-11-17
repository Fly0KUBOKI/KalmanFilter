function [p, v, q, ba, bg] = integrate_nominal(p, v, q, ba, bg, a_meas, w_meas, dt, g, gyro_noise_threshold, accel_noise_threshold)
% INTEGRATE_NOMINAL  ノミナル状態の数値積分（RK2, 台形則）
% a_meas, w_meas は生センサ（機体座標系）
% gyro_noise_threshold  - 角速度の閾値（3x1ベクトル or スカラー）
% accel_noise_threshold - 加速度の閾値（3x1ベクトル or スカラー）

% バイアス補正
a = a_meas - ba;
% 【標準IMU仕様】w_meas = [roll_rate, pitch_rate, yaw_rate] (航空機座標系)
w = w_meas - bg;

% 加速度の閾値処理: 各軸のノイズレベル以下の成分を0にする
for i = 1:3
    if abs(a(i)) < accel_noise_threshold(i) * dt
        a(i) = 0;
    end
end

% 角速度の閾値処理: **テスト用に無効化**
w_dt = w * dt;
% ジャイロ積分を確実に動作させるため、閾値処理をコメントアウト
% for i = 1:3
%     if abs(w_dt(i)) < gyro_noise_threshold(i) * dt *0.1
%         w_dt(i) = 0;
%     end
% end

% ジャイロ積分（クォータニオン更新）を有効化
% 角速度 w を dt で積分し、四元数を更新する。小角度では Taylor 展開で安定化。
w_dt_norm = norm(w_dt);
threshold = 1e-15;  % 極微小角速度も積分するための閾値

if w_dt_norm > threshold
    % 小さな回転ベクトルを四元数に変換
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

    q = quat_lib('quatmultiply', q, delta_q);

    % 四元数正規化（数値誤差を抑える）
    q_norm = norm(q);
    if abs(q_norm - 1.0) > 1e-12
        q = q / q_norm;
    end
end

% 回転行列 body -> world
Rb = quat_lib('quat_to_rotm', q);

% 世界座標系での比力
% 注意: a_meas は「比力」(specific force = a_true - g)
%       バイアス補正後の a も比力
%       Rb * a で世界座標系の比力が得られる
a_world = Rb * a;

% 速度・位置更新: Simpson の考察に基づく2次精度手法 (Adams-Bashforth2 相当)
% 解説:
%  - 真の Simpson 法を1ステップで適用するには a_{n+1} が必要だが、実運用では未知。
%  - a_{n+1} を線形外挿 (a_{n+1} ≈ 2 a_n - a_{n-1}) すると、Simpson の公式は
%    v_{n+1} = v_n + dt*(3/2 a_n - 1/2 a_{n-1}) となり、Adams-Bashforth 2 と同値になる。
%  - ここではこの安定で2次精度の多段法を採用する（初回ステップのみ1次で初期化）。

persistent prev_a_world prev_v prev_initialized
if isempty(prev_initialized)
    % 初回ステップ: 単純台形/オイラーで初期化
    v_prev = v;
    v_candidate = v + (a_world + g) * dt; % forward Euler-ish
    % 速度発散防止
    max_accel = 2.0;
    dv = v_candidate - v;
    dv_norm = norm(dv);
    max_dv = max_accel * dt;
    if dv_norm > max_dv
        dv = dv * (max_dv / dv_norm);
    end
    v = v + dv;
    % 速度クリップ
    max_velocity = 50;
    v = max(min(v, max_velocity), -max_velocity);

    % 位置は2次近似のために加速度を用いた予測を採用
    p = p + v * dt + 0.5 * (a_world + g) * dt * dt;

    % 保存
    prev_a_world = a_world;
    prev_v = v_prev;
    prev_initialized = true;
else
    % AB2 (Simpson 外挿相当): v_{n+1} = v_n + dt*(3/2*a_n - 1/2*a_{n-1})
    a_prev = prev_a_world;
    v_old = v; % v_n
    v_new = v + dt * (1.5 * (a_world + g) - 0.5 * (a_prev + g));

    % 速度発散防止
    max_accel = 2.0;
    dv = v_new - v;
    dv_norm = norm(dv);
    max_dv = max_accel * dt;
    if dv_norm > max_dv
        dv = dv * (max_dv / dv_norm);
    end
    v = v + dv;

    % 速度クリップ
    max_velocity = 50;
    v = max(min(v, max_velocity), -max_velocity);

    % 位置更新も AB2 を適用: p_{n+1} = p_n + dt*(3/2*v_n - 1/2*v_{n-1})
    p = p + dt * (1.5 * v_old - 0.5 * prev_v);

    % 保存
    prev_v = v_old;
    prev_a_world = a_world;
end

end