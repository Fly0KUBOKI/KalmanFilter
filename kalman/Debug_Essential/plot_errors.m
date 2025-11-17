function plot_errors()
% PLOT_ERRORS - エラーの時系列プロット
%
% 使用法:
%   plot_errors()
%
% 必要ファイル:
%   - Results/estimation.csv
%   - GenerateData/truth_data.csv
%
% 出力:
%   3つのサブプロット (Position, Velocity, Attitude)

    %% ファイル読み込み
    try
        est_data = readtable('Results/estimation.csv');
        truth_data = readtable('GenerateData/truth_data.csv');
    catch ME
        error('ファイル読み込み失敗: %s', ME.message);
    end
    
    %% データサイズ確認
    n = min(height(est_data), height(truth_data));
    time = truth_data.time(1:n);
    
    %% エラー計算
    % Position
    pos_error_x = est_data.px(1:n) - truth_data.x(1:n);
    pos_error_y = est_data.py(1:n) - truth_data.y(1:n);
    pos_error_z = est_data.pz(1:n) - truth_data.z(1:n);
    pos_error_norm = sqrt(pos_error_x.^2 + pos_error_y.^2 + pos_error_z.^2);
    
    % Velocity
    vel_error_x = est_data.vx(1:n) - truth_data.vx(1:n);
    vel_error_y = est_data.vy(1:n) - truth_data.vy(1:n);
    vel_error_z = est_data.vz(1:n) - truth_data.vz(1:n);
    vel_error_norm = sqrt(vel_error_x.^2 + vel_error_y.^2 + vel_error_z.^2);
    
    % Attitude
    roll_error = wrapTo180(est_data.roll(1:n) - truth_data.roll(1:n));
    pitch_error = wrapTo180(est_data.pitch(1:n) - truth_data.pitch(1:n));
    yaw_error = wrapTo180(est_data.yaw(1:n) - truth_data.yaw(1:n));
    
    %% プロット作成
    figure('Name', 'ESKF Estimation Errors', 'NumberTitle', 'off', ...
        'Position', [100, 100, 1200, 800]);
    
    % Position Error
    subplot(3,1,1);
    hold on;
    plot(time, pos_error_x, 'r-', 'LineWidth', 1.2, 'DisplayName', 'X');
    plot(time, pos_error_y, 'g-', 'LineWidth', 1.2, 'DisplayName', 'Y');
    plot(time, pos_error_z, 'b-', 'LineWidth', 1.2, 'DisplayName', 'Z');
    plot(time, pos_error_norm, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Norm');
    hold off;
    grid on;
    xlabel('Time [s]');
    ylabel('Position Error [m]');
    title('Position Estimation Error');
    legend('Location', 'best');
    
    % Velocity Error
    subplot(3,1,2);
    hold on;
    plot(time, vel_error_x, 'r-', 'LineWidth', 1.2, 'DisplayName', 'X');
    plot(time, vel_error_y, 'g-', 'LineWidth', 1.2, 'DisplayName', 'Y');
    plot(time, vel_error_z, 'b-', 'LineWidth', 1.2, 'DisplayName', 'Z');
    plot(time, vel_error_norm, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Norm');
    hold off;
    grid on;
    xlabel('Time [s]');
    ylabel('Velocity Error [m/s]');
    title('Velocity Estimation Error');
    legend('Location', 'best');
    
    % Attitude Error
    subplot(3,1,3);
    hold on;
    plot(time, roll_error, 'r-', 'LineWidth', 1.2, 'DisplayName', 'Roll');
    plot(time, pitch_error, 'g-', 'LineWidth', 1.2, 'DisplayName', 'Pitch');
    plot(time, yaw_error, 'b-', 'LineWidth', 1.2, 'DisplayName', 'Yaw');
    hold off;
    grid on;
    xlabel('Time [s]');
    ylabel('Attitude Error [deg]');
    title('Attitude Estimation Error');
    legend('Location', 'best');
    
    %% 統計情報表示
    fprintf('\n--- グラフ表示完了 ---\n');
    fprintf('Position RMS: %.4f m\n', rms(pos_error_norm));
    fprintf('Velocity RMS: %.4f m/s\n', rms(vel_error_norm));
    fprintf('Roll RMS:     %.4f deg\n', rms(roll_error));
    fprintf('Pitch RMS:    %.4f deg\n', rms(pitch_error));
    fprintf('Yaw RMS:      %.4f deg\n', rms(yaw_error));
end
