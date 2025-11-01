function plot_yaw(truth, estimation, static_samples)
% plot_yaw - Yaw角の比較プロット

start_k = static_samples + 1;
end_k = length(estimation.time);
time_range = estimation.time(start_k:end_k);

figure('Name', 'Yaw角の比較');
plot(time_range, truth.yaw(start_k:end_k), 'b-', 'LineWidth', 1.5);
hold on;
plot(time_range, estimation.yaw(start_k:end_k), 'r--', 'LineWidth', 1.5);
hold off;
grid on;
xlabel('時刻 [s]');
ylabel('Yaw [deg]');
title('姿勢（Yaw角）の比較');
legend({'真値','推定'}, 'Location','best');
end
