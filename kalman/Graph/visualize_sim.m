function visualize_sim(t, true_state, meas, x_est, P_est)
N = numel(t);
figure('Name','Position Tracking');
subplot(2,2,1);
plot(true_state(:,1), true_state(:,2), '-k','LineWidth',2.5,'DisplayName','True'); hold on;
plot(meas.pos(:,1), meas.pos(:,2), '.r','DisplayName','Measured');
plot(x_est(:,1), x_est(:,2), '-b','LineWidth',0.8,'DisplayName','Kalman');
legend; xlabel('x [m]'); ylabel('y [m]'); axis equal; grid on;
title('Trajectory');
subplot(2,2,2);
plot(t, true_state(:,1), '-k', 'LineWidth', 2.5); hold on;
plot(t, meas.pos(:,1), '.r');
plot(t, x_est(:,1), '-b', 'LineWidth', 0.8);
legend('True x','Measured x','Kalman x'); xlabel('Time [s]'); ylabel('x [m]'); grid on;
subplot(2,2,3);
plot(t, true_state(:,2), '-k', 'LineWidth', 2.5); hold on;
plot(t, meas.pos(:,2), '.r');
plot(t, x_est(:,2), '-b', 'LineWidth', 0.8);
legend('True y','Measured y','Kalman y'); xlabel('Time [s]'); ylabel('y [m]'); grid on;
subplot(2,2,4);
sigma_x = squeeze(sqrt(squeeze(P_est(1,1,:))));
sigma_y = squeeze(sqrt(squeeze(P_est(2,2,:))));
plot(t, sigma_x, '-b', t, sigma_y, '-r');
legend('\sigma_x','\sigma_y'); xlabel('Time [s]'); ylabel('Std [m]'); grid on;
title('State uncertainty (1\sigma)');
figure('Name','Animation');
axis equal; grid on; hold on;
minx = min([true_state(:,1); meas.pos(:,1); x_est(:,1)])-5;
maxx = max([true_state(:,1); meas.pos(:,1); x_est(:,1)])+5;
miny = min([true_state(:,2); meas.pos(:,2); x_est(:,2)])-5;
maxy = max([true_state(:,2); meas.pos(:,2); x_est(:,2)])+5;
xlim([minx maxx]); ylim([miny maxy]);
h_true = plot(true_state(1,1), true_state(1,2), 'ok','MarkerFaceColor','k','MarkerSize',10);
h_meas = plot(meas.pos(1,1), meas.pos(1,2), '.r','MarkerSize',8);
h_est = plot(x_est(1,1), x_est(1,2), 'ob','MarkerFaceColor','b','MarkerSize',6);
for k=1:round(N/200):N
    set(h_true, 'XData', true_state(k,1), 'YData', true_state(k,2));
    set(h_meas, 'XData', meas.pos(k,1), 'YData', meas.pos(k,2));
    set(h_est, 'XData', x_est(k,1), 'YData', x_est(k,2));
    drawnow;
end

% Figure 7: GPS raw data on x,y plane (ユーザー要望で図7をGPSのraw軌跡表示に変更)
figure(7); clf; hold on; grid on; axis equal;
    % prefer explicit GPS measurements if present
    gps_xy = [];
    if isfield(meas,'gps') && ~isempty(meas.gps)
        gps_xy = meas.gps;
    elseif isfield(meas,'pos') && ~isempty(meas.pos)
        % fallback: use measured position
        gps_xy = meas.pos;
    end
    if ~isempty(gps_xy)
        % plot as points on x-y plane (no connecting lines)
        plot(true_state(:,1), true_state(:,2), '.k', 'MarkerSize',8, 'DisplayName','True'); hold on;
        plot(gps_xy(:,1), gps_xy(:,2), '.','Color',[0.85 0.33 0.1], 'MarkerSize',8, 'DisplayName','GPS raw');
        plot(x_est(:,1), x_est(:,2), '.b', 'MarkerSize',6, 'DisplayName','Kalman');
        legend; xlabel('x [m]'); ylabel('y [m]'); title('GPS raw trajectory (Figure 7) - points');
    else
        text(0.5,0.5,'No GPS/pos data available','Units','normalized','HorizontalAlignment','center');
        title('GPS raw trajectory (Figure 7) - no data');
    end
end
