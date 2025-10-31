function [vel_world, vel_body] = compute_body_velocities(pos_world, vel_world, attitude, params, heading_mode)
% COMPUTE_BODY_VELOCITIES 速度の補完とボディフレーム速度の計算
%
% 入力:
%   pos_world - 世界座標系での位置 [N x 3]
%   vel_world - 世界座標系での速度 [N x 3]
%   attitude - 姿勢（ロール、ピッチ、ヨー） [N x 3]
%   params - 設定パラメータ
%   heading_mode - ヘディングモード
% 出力:
%   vel_world - 補完された世界座標系での速度 [N x 3]
%   vel_body - ボディフレーム速度 [N x 3]

dt = params.dt;
motion_type = params.motion_type;
N = size(pos_world, 1);

% 速度の差分補完
for i = 1:N
    if i == 1
        if all(vel_world(1,:) == 0) && N > 1
            vel_world(1,:) = (pos_world(2,:) - pos_world(1,:)) / dt;
        end
    else
        if all(vel_world(i,:) == 0)
            vel_world(i,:) = (pos_world(i,:) - pos_world(i-1,:)) / dt;
        end
    end
end

% ボディフレーム速度の計算
vel_body = zeros(N,3);

for i = 1:N
    vx = vel_world(i,1); 
    vy = vel_world(i,2);

    % ヘディングモードに応じた姿勢調整
    if strcmp(heading_mode, 'fixed_north')
        attitude(i,1:2) = [0,0];
        attitude(i,3) = 0;  % yaw = 0 (固定北向き)
    elseif strcmp(motion_type, 'circular')
        % 円運動の場合は既に計算済み
    else
        % その他の場合は速度から計算
        yaw = atan2(-vx, vy);  % 時計回りが負になるよう調整
        yaw = atan2(sin(yaw), cos(yaw));  % [-pi, pi]にラップ
        attitude(i,1:2) = [0,0];
        attitude(i,3) = yaw;
    end

    % 回転行列
    R = eul2rotm([attitude(i,3), attitude(i,2), attitude(i,1)], 'ZYX');
    
    if strcmp(heading_mode, 'align_velocity')
        % align_velocityモード：ボディフレーム速度は[0, |v|, 0]
        speed = hypot(vx, vy);
        vel_body(i,:) = [0, speed, 0];
    else
        vel_body(i,:) = (R' * vel_world(i,:)')';
    end
end

end