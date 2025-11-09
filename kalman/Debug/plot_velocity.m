function plot_velocity(truth, estimation, static_samples)
% plot_velocity - 速度成分（vx, vy, vz）の比較プロット（各成分を独立表示）

start_k = static_samples + 1;
end_k = length(estimation.time);
time_range = estimation.time(start_k:end_k);

% Estimation components
vx_est = estimation.vx(start_k:end_k);
vy_est = estimation.vy(start_k:end_k);
vz_est = estimation.vz(start_k:end_k);

% Truth components (if available)
vx_true = interp1(truth.time, truth.vx, time_range, 'linear', NaN);
vy_true = interp1(truth.time, truth.vy, time_range, 'linear', NaN);
vz_true = interp1(truth.time, truth.vz, time_range, 'linear', NaN);

% Plot vx
figure('Name', 'Velocity X (vx)');
plot(time_range, vx_est, '-','Color',[0 0 1],'LineWidth',1.5); hold on; % x: blue
plot(time_range, vx_true, '--','Color',[0 0 1],'LineWidth',1.2);
hold off; grid on; xlabel('時刻 [s]'); ylabel('vx [m/s]');
legend('vx\_est','vx\_truth','Location','best');

% Plot vy
figure('Name', 'Velocity Y (vy)');
plot(time_range, vy_est, '-','Color',[1 0 0],'LineWidth',1.5); hold on; % y: red
plot(time_range, vy_true, '--','Color',[1 0 0],'LineWidth',1.2);
hold off; grid on; xlabel('時刻 [s]'); ylabel('vy [m/s]');
legend('vy\_est','vy\_truth','Location','best');

% Plot vz (altitude rate)
figure('Name', 'Velocity Z (vz)');
plot(time_range, vz_est, '-','Color',[1 0.75 0],'LineWidth',1.5); hold on; % z: yellow-ish
plot(time_range, vz_true, '--','Color',[1 0.75 0],'LineWidth',1.2);
hold off; grid on; xlabel('時刻 [s]'); ylabel('vz [m/s]');
legend('vz\_est','vz\_truth','Location','best');
end
