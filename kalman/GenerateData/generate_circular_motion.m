function [pos_world, vel_world, attitude] = generate_circular_motion(params, t, N)
% GENERATE_CIRCULAR_MOTION 円運動の位置、速度、姿勢を生成
%
% 入力:
%   params - 設定パラメータ
%   t - 時間ベクトル
%   N - サンプル数
% 出力:
%   pos_world - 世界座標系での位置 [N x 3]
%   vel_world - 世界座標系での速度 [N x 3] 
%   attitude - 姿勢（ロール、ピッチ、ヨー） [N x 3]

% パラメータ取得
radius = params.motion.circular.radius;
omega = params.motion.circular.omega;
altitude = params.motion.circular.altitude;

% 静止時間とソフトスタート時間
static_time = 5;
if isfield(params, 'static_time')
    static_time = params.static_time;
end
accel_time = 5;
if isfield(params.motion.circular, 'accel_time')
    accel_time = params.motion.circular.accel_time;
end

dt = params.dt;

% 初期化
pos_world = zeros(N,3);
vel_world = zeros(N,3);
attitude = zeros(N,3);

% 円運動の中心と初期位置
center_x = radius;
center_y = 0;
theta = pi;  % 初期角度（西方向）
theta_dot = 0;  % 初期角速度

% 円運動生成
for i = 1:N
    % 角速度スケーリング（静止期間 + ソフトスタート）
    if t(i) < static_time
        omega_scale = 0.0;  % 静止期間
    elseif t(i) < static_time + accel_time
        t_motion = t(i) - static_time;
        omega_scale = 0.5 * (1 - cos(pi * t_motion / accel_time));  % ソフトスタート
    else
        omega_scale = 1.0;  % 通常運動
    end

    % 現在の角速度（時計回り：負）
    theta_dot_cur = -omega * omega_scale;

    % 角度積分（台形則）
    if i > 1
        theta = theta + 0.5 * (theta_dot + theta_dot_cur) * dt;
    end
    theta_dot = theta_dot_cur;

    % 位置計算
    pos_world(i,1) = center_x + radius * cos(theta);  % East
    pos_world(i,2) = center_y + radius * sin(theta);  % North
    pos_world(i,3) = altitude;  % Up

    % 速度計算
    vel_world(i,1) = -radius * theta_dot * sin(theta);  % East
    vel_world(i,2) =  radius * theta_dot * cos(theta);  % North
    vel_world(i,3) = 0;

    % 姿勢計算
    attitude(i,1:2) = [0,0];  % roll, pitch = 0
    attitude(i,3) = wrapToPi(theta - pi);  % yaw（時計回り）
    if mod(i, 10000) == 0
        fprintf('Motion step %d / %d\n', i, N);
    end
end

end