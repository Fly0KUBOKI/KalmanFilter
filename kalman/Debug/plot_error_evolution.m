function plot_error_evolution(error_evolution, divergence_info)
% plot_error_evolution - 誤差の時間発展プロット（対数スケール）

figure('Name', '誤差の時間発展');
semilogy(error_evolution.time, error_evolution.pos_error, 'b-', 'LineWidth', 1.5);
hold on;
semilogy(error_evolution.time, error_evolution.vel_error, 'r-', 'LineWidth', 1.5);
semilogy(error_evolution.time, error_evolution.att_error, 'g-', 'LineWidth', 1.5);

if divergence_info.pos_diverge_time < inf
    xline(divergence_info.pos_diverge_time, 'b--', '位置発散');
end
if divergence_info.vel_diverge_time < inf
    xline(divergence_info.vel_diverge_time, 'r--', '速度発散');
end
if divergence_info.att_diverge_time < inf
    xline(divergence_info.att_diverge_time, 'g--', '姿勢発散');
end

hold off;
grid on;
xlabel('時刻 [s]');
ylabel('誤差 (対数スケール)');
title('誤差の時間発展');
legend({'位置誤差 [m]','速度誤差 [m/s]','姿勢誤差 [deg]'}, 'Location','best');
end
