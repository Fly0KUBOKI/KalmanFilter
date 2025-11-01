function divergence_info = divergence_diagnosis(error_evolution, dt)
% divergence_diagnosis - 発散を診断

n_points = length(error_evolution.time);

divergence_info.pos_growth_rate = zeros(n_points-1, 1);
divergence_info.vel_growth_rate = zeros(n_points-1, 1);
divergence_info.att_growth_rate = zeros(n_points-1, 1);

for i = 1:n_points-1
    dt_interval = error_evolution.time(i+1) - error_evolution.time(i);
    
    divergence_info.pos_growth_rate(i) = ...
        (error_evolution.pos_error(i+1) - error_evolution.pos_error(i)) / dt_interval;
    divergence_info.vel_growth_rate(i) = ...
        (error_evolution.vel_error(i+1) - error_evolution.vel_error(i)) / dt_interval;
    divergence_info.att_growth_rate(i) = ...
        (error_evolution.att_error(i+1) - error_evolution.att_error(i)) / dt_interval;
end

pos_threshold = 0.01;
vel_threshold = 0.01;
att_threshold = 0.1;

pos_diverge_idx = find(divergence_info.pos_growth_rate > pos_threshold, 1);
vel_diverge_idx = find(divergence_info.vel_growth_rate > vel_threshold, 1);
att_diverge_idx = find(divergence_info.att_growth_rate > att_threshold, 1);

fprintf('\n--- 発散の診断 ---\n');
fprintf('増加率の閾値:\n');
fprintf('  位置: %.4f m/s\n', pos_threshold);
fprintf('  速度: %.4f (m/s)/s\n', vel_threshold);
fprintf('  姿勢: %.4f deg/s\n\n', att_threshold);

if ~isempty(pos_diverge_idx)
    t_diverge = error_evolution.time(pos_diverge_idx);
    fprintf('位置誤差の発散開始: t=%.4f秒\n', t_diverge);
    divergence_info.pos_diverge_time = t_diverge;
else
    fprintf('位置誤差: 発散なし\n');
    divergence_info.pos_diverge_time = inf;
end

if ~isempty(vel_diverge_idx)
    t_diverge = error_evolution.time(vel_diverge_idx);
    fprintf('速度誤差の発散開始: t=%.4f秒\n', t_diverge);
    divergence_info.vel_diverge_time = t_diverge;
else
    fprintf('速度誤差: 発散なし\n');
    divergence_info.vel_diverge_time = inf;
end

if ~isempty(att_diverge_idx)
    t_diverge = error_evolution.time(att_diverge_idx);
    fprintf('姿勢誤差の発散開始: t=%.4f秒\n', t_diverge);
    divergence_info.att_diverge_time = t_diverge;
else
    fprintf('姿勢誤差: 発散なし\n');
    divergence_info.att_diverge_time = inf;
end

fprintf('\n平均誤差増加率:\n');
fprintf('  位置: %.6f m/s\n', mean(divergence_info.pos_growth_rate));
fprintf('  速度: %.6f (m/s)/s\n', mean(divergence_info.vel_growth_rate));
fprintf('  姿勢: %.6f deg/s\n', mean(divergence_info.att_growth_rate));
end
