function [pos_world, vel_world, attitude] = generate_random_walk(params, t, N)
% GENERATE_RANDOM_WALK ランダムウォーク運動の位置、速度、姿勢を生成
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
vel_std = params.motion.random_walk.velocity_std;
ang_std = params.motion.random_walk.angular_std;

% 静止時間
static_time = 5;
if isfield(params, 'static_time')
    static_time = params.static_time;
end

dt = params.dt;

% 初期化
pos_world = zeros(N,3);
vel_world = zeros(N,3);
attitude = zeros(N,3);

% 初期条件
pos_world(1,:) = [0, 0, 0];
v_forward = params.initial.velocity(1);
yaw = 0;

% ランダムシード設定
rng(42);

% ランダムウォーク生成
for i = 2:N
    if t(i) < static_time
        % 静止期間：位置・速度・姿勢を維持
        pos_world(i,:) = pos_world(1,:);
        vel_world(i,:) = [0, 0, 0];
        attitude(i,:) = [0, 0, 0];
    else
        % 運動期間：ランダムウォーク
        dv = randn() * vel_std * sqrt(dt);
        v_forward = max(0, v_forward + dv);
        yaw = yaw + randn() * ang_std * dt;
        
        vel_world(i,1) = v_forward * cos(yaw);
        vel_world(i,2) = v_forward * sin(yaw);
        vel_world(i,3) = 0;
        
        pos_world(i,:) = pos_world(i-1,:) + vel_world(i,:) * dt;
        attitude(i,:) = [0, 0, yaw];
    end
    if mod(i, 10000) == 0
        fprintf('Motion step %d / %d\n', i, N);
    end
end

end