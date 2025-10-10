function vel_avg = calculate_velocity_average(vel_buffer)
% 速度推定結果の平均を計算
vel_sum = zeros(3, 1, 'single');
accel_sum = zeros(3, 1, 'single');
velP_sum = zeros(3,3, 'single');
count = 0;
for i = 1:length(vel_buffer)
    if ~isempty(vel_buffer{i})
        vel_sum = vel_sum + vel_buffer{i}.velocity;
        % vel_buffer entries include acceleration estimate
        if isfield(vel_buffer{i}, 'acceleration') && ~isempty(vel_buffer{i}.acceleration)
            accel_sum = accel_sum + vel_buffer{i}.acceleration;
        end
        if isfield(vel_buffer{i}, 'P') && ~isempty(vel_buffer{i}.P)
            % P is 6x6; take velocity part (1:3,1:3)
            Pvel = single(vel_buffer{i}.P(1:3,1:3));
            velP_sum = velP_sum + Pvel;
        end
        count = count + 1;
    end
end
if count > 0
    vel_avg = struct();
    vel_avg.velocity = vel_sum / count;
    % 平均加速度を返す（以前はゼロ固定だった）
    vel_avg.acceleration = accel_sum / count;
    % 平均共分散（速度部分）を返す
    vel_avg.P = velP_sum / count;
else
    vel_avg = [];
end
end
