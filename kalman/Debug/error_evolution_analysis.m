function error_evolution = error_evolution_analysis(estimation, truth, static_samples, dt)
% error_evolution_analysis - 誤差の時間発展を分析

start_k = static_samples + 1;
end_k = length(estimation.time);

% 対数的にサンプリング
sample_points = unique(round(logspace(log10(start_k), log10(end_k), 20)));

n_points = length(sample_points);
error_evolution.time = zeros(n_points, 1);
error_evolution.pos_error = zeros(n_points, 1);
error_evolution.vel_error = zeros(n_points, 1);
error_evolution.att_error = zeros(n_points, 1);
error_evolution.sample_k = sample_points;

fprintf('--- 誤差の時間発展 ---\n');
fprintf('時刻[s]\t\t位置誤差[m]\t速度誤差[m/s]\t姿勢誤差[deg]\n');
fprintf('----------------------------------------------------------\n');

for i = 1:n_points
    k = sample_points(i);
    
    error_evolution.time(i) = estimation.time(k);
    
    pos_err = norm([estimation.px(k) - truth.x(k), ...
                   estimation.py(k) - truth.y(k), ...
                   estimation.pz(k) - truth.z(k)]);
    
    vel_err = norm([estimation.vx(k) - truth.vx(k), ...
                   estimation.vy(k) - truth.vy(k), ...
                   estimation.vz(k) - truth.vz(k)]);
    
    att_err = norm([estimation.roll(k) - truth.roll(k), ...
                   estimation.pitch(k) - truth.pitch(k), ...
                   estimation.yaw(k) - truth.yaw(k)]);
    
    error_evolution.pos_error(i) = pos_err;
    error_evolution.vel_error(i) = vel_err;
    error_evolution.att_error(i) = att_err;
    
    if i <= 10 || mod(i, 5) == 0
        fprintf('%.4f\t\t%.6f\t%.6f\t%.6f\n', ...
            error_evolution.time(i), pos_err, vel_err, att_err);
    end
end

k = end_k;
t = estimation.time(k);
pos_err = norm([estimation.px(k) - truth.x(k), ...
               estimation.py(k) - truth.y(k), ...
               estimation.pz(k) - truth.z(k)]);
vel_err = norm([estimation.vx(k) - truth.vx(k), ...
               estimation.vy(k) - truth.vy(k), ...
               estimation.vz(k) - truth.vz(k)]);
att_err = norm([estimation.roll(k) - truth.roll(k), ...
               estimation.pitch(k) - truth.pitch(k), ...
               estimation.yaw(k) - truth.yaw(k)]);

fprintf('%.4f\t\t%.6f\t%.6f\t%.6f (最終)\n', t, pos_err, vel_err, att_err);
end
