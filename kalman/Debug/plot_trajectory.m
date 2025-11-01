function plot_trajectory(truth, estimation)
% plot_trajectory - 位置の軌跡プロット

figure('Name', '位置の軌跡');
plot(truth.x, truth.y, 'b-', 'LineWidth', 1.5);
hold on;
plot(estimation.px, estimation.py, 'r--', 'LineWidth', 1.5);
hold off;
grid on;
axis equal;
xlabel('X [m]');
ylabel('Y [m]');
title('位置の軌跡（XY平面）');
legend({'真値','推定'}, 'Location','best');
end
