function plot_yaw(truth, estimation, static_samples)
% plot_yaw - Yaw角の比較プロット（真値は点線、推定は実線）

start_k = static_samples + 1;
end_k = length(estimation.time);
time_range = estimation.time(start_k:end_k);

% Align truth yaw to estimation time if truth has its own time vector
if isfield(truth, 'time')
	tr_yaw = interp1(truth.time, truth.yaw, time_range, 'linear', NaN);
else
	tr_yaw = truth.yaw(start_k:end_k);
end

figure('Name', 'Yaw [deg]');
plot(time_range, estimation.yaw(start_k:end_k), '-k', 'LineWidth', 1.5); hold on; % estimation: solid
plot(time_range, tr_yaw, '--k', 'LineWidth', 1.2); % truth: dashed
hold off; grid on;
xlabel('時刻 [s]');
ylabel('Yaw [deg]');
title('Yaw の比較');
legend({'yaw\_est','yaw\_truth'}, 'Location','best');
end
