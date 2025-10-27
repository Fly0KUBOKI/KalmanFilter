function [p, v, q, ba, bg, P] = update_gps(p, v, q, ba, bg, P, lat, lon, alt, origin, gps_dt)
    % UPDATE_GPS  GPS位置観測による位置・速度の同時更新(標準ESKFアプローチ)
    %
    % GPS更新頻度が低い場合、差分速度を直接観測すると遅れが生じるため、
    % 位置のみを観測値とし、共分散行列を通じて速度も間接的に修正する。
    %
    % 入力:
    %   p, v, q, ba, bg, P - 現在の公称状態と共分散行列
    %   lat, lon, alt - GPS観測値(緯度、経度、高度)
    %   origin - 原点 [lat0; lon0; alt0]
    %   gps_dt - GPS更新間隔(使用しない、互換性のため保持)
    %
    % 出力:
    %   p, v, q, ba, bg, P - 更新後の状態と共分散行列
    
    lat0 = origin(1); lon0 = origin(2); alt0 = origin(3);

    % 緯度経度をローカルメートル座標に変換
    % 緯度(lat) → North(y), 経度(lon) → East(x)
    % データ生成時の変換と一致させる:
    %   dlat = north_m * 9.0e-6
    %   dlon = east_m * 9.0e-6 / cosd(lat)
    y = (lat - lat0) / (9.0e-6);                    % North [m]
    x = (lon - lon0) / (9.0e-6 / cosd(lat0));       % East [m]
    z = alt - alt0;                                  % Up [m]

    z_gps = [x; y; z];

    % 観測モデル: 位置のみを観測(3x15の観測行列)
    % H = [I_3x3, 0_3x12]
    H = [eye(3), zeros(3,12)];
    z_meas = z_gps;
    h = p;  % 予測位置

    % 観測ノイズ共分散行列の初期値(位置のみ、3x3)
    % GPS精度に応じて調整(例: 水平5m, 垂直10m)
    R0 = diag([5.0, 5.0, 10.0]);
    y0 = z_meas - h;

    % 適応的R更新(オプション)
    params = adaptive_R_update(struct(), y0, H, P, R0, {struct('name','gps','range',1:3)});

    if isfield(params,'kf') && isfield(params.kf,'R_est') && isfield(params.kf.R_est,'gps')
        R_est = diag(params.kf.R_est.gps);
    else
        R_est = R0;
    end

    % イノベーションとイノベーション共分散の計算
    [yv, S, R_used] = compute_innovation_and_S(z_meas, h, H, P, R_est, struct());
    
    % カルマンゲインの計算
    K = compute_kalman_gain(P, H, S);
    
    % 誤差状態の推定
    % dx = [δp; δv; δθ; δba; δbg] (15x1)
    % 位置観測から、共分散P経由で速度誤差も推定される
    dx = K * yv;

    % 状態の更新
    % 位置の修正
    p = p + dx(1:3);
    
    % 速度の修正(位置観測から間接的に推定された速度誤差)
    v = v + dx(4:6);
    
    % 姿勢の修正(クォータニオン更新)
    dtheta = dx(7:9);
    dq = [1; 0.5 * dtheta];  % 小角度近似
    dq = dq / norm(dq);       % 正規化
    q = quat_multiply(q, dq);
    q = q / norm(q);          % 再正規化
    
    % バイアスの修正
    ba = ba + dx(10:12);
    bg = bg + dx(13:15);

    % 共分散行列の更新
    x_pred = zeros(15,1);
    [~, P] = update_state_covariance(x_pred, P, K, H, yv, R_used);
end

function q_out = quat_multiply(q1, q2)
    % クォータニオンの乗算 q_out = q1 ⊗ q2
    w1 = q1(1); v1 = q1(2:4);
    w2 = q2(1); v2 = q2(2:4);
    
    w_out = w1*w2 - dot(v1, v2);
    v_out = w1*v2 + w2*v1 + cross(v1, v2);
    
    q_out = [w_out; v_out];
end