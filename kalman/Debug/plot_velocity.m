function plot_velocity(truth, estimation, static_samples)
% plot_velocity - 速度ノルムの比較プロット

start_k = static_samples + 1;
end_k = length(estimation.time);
time_range = estimation.time(start_k:end_k);

vel_true_norm = sqrt(truth.vx(start_k:end_k).^2 + truth.vy(start_k:end_k).^2);
vel_est_norm = sqrt(estimation.vx(start_k:end_k).^2 + estimation.vy(start_k:end_k).^2);

figure('Name', '速度ノルムの比較');
plot(time_range, vel_true_norm, 'b-', 'LineWidth', 1.5);
hold on;
plot(time_range, vel_est_norm, 'r--', 'LineWidth', 1.5);
hold off;
grid on;
xlabel('時刻 [s]');
ylabel('速度ノルム [m/s]');
title('速度ノルムの比較');
legend({'真値','推定'}, 'Location','best');
end
