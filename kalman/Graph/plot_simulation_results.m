function plot_simulation_results(results_file)
% PLOT_SIMULATION_RESULTS - シミュレーション結果の詳細プロット
% Graphフォルダ用の独立したプロット関数
%
% Input:
%   results_file: 保存された結果ファイルのパス（オプション）

if nargin < 1
    % デフォルトの結果ファイルパスを設定
    results_file = fullfile('..', 'KalmanFilter', 'simulation_results.mat');
end

% 結果ファイルが存在しない場合は、直接シミュレーションを実行
if ~exist(results_file, 'file')
    fprintf('結果ファイルが見つかりません。シミュレーションを実行します...\n');
    
    % KalmanFilterフォルダにパスを追加
    addpath(fullfile('..', 'KalmanFilter'));
    
    % シミュレーション実行
    run_simulation();
    return;
end

% 結果を読み込み
fprintf('結果ファイルを読み込み中: %s\n', results_file);
load(results_file, 'vel_results', 'att_results', 'pos_results', 'truth_data', 'sensor_data');

% プロット作成
create_detailed_plots(vel_results, att_results, pos_results, truth_data, sensor_data);

fprintf('プロット完了!\n');

end

function create_detailed_plots(vel_results, att_results, pos_results, truth_data, sensor_data)
% 詳細プロット作成

% 1. 速度推定結果の詳細プロット
figure('Name', '速度推定詳細結果', 'Position', [100, 600, 1200, 800]);

% 速度成分の時系列
subplot(2, 2, 1);
hold on;
title('速度成分時系列 (Vx, Vy, Vz)');
xlabel('時間 [s]');
ylabel('速度 [m/s]');
grid on;

% 真値
plot(truth_data.t, truth_data.vel(:, 1), 'k-', 'LineWidth', 1.5, 'DisplayName', 'Vx真値');
plot(truth_data.t, truth_data.vel(:, 2), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Vy真値');
plot(truth_data.t, truth_data.vel(:, 3), 'k:', 'LineWidth', 1.5, 'DisplayName', 'Vz真値');

% 推定値
t_vel = [];
vx_est = [];
vy_est = [];
vz_est = [];

for i = 1:length(vel_results)
    if ~isempty(vel_results{i})
        t_vel = [t_vel; truth_data.t(i)];
        vx_est = [vx_est; vel_results{i}.velocity(1)];
        vy_est = [vy_est; vel_results{i}.velocity(2)];
        vz_est = [vz_est; vel_results{i}.velocity(3)];
    end
end

if ~isempty(t_vel)
    plot(t_vel, vx_est, 'b-', 'LineWidth', 1, 'DisplayName', 'Vx推定');
    plot(t_vel, vy_est, 'r-', 'LineWidth', 1, 'DisplayName', 'Vy推定');
    plot(t_vel, vz_est, 'g-', 'LineWidth', 1, 'DisplayName', 'Vz推定');
end

legend('Location', 'best');

% 速度誤差
subplot(2, 2, 2);
hold on;
title('速度推定誤差');
xlabel('時間 [s]');
ylabel('誤差 [m/s]');
grid on;

if ~isempty(t_vel)
    vx_error = abs(vx_est - truth_data.vel(1:length(vx_est), 1));
    vy_error = abs(vy_est - truth_data.vel(1:length(vy_est), 2));
    vz_error = abs(vz_est - truth_data.vel(1:length(vz_est), 3));
    
    plot(t_vel, vx_error, 'b-', 'DisplayName', 'Vx誤差');
    plot(t_vel, vy_error, 'r-', 'DisplayName', 'Vy誤差');
    plot(t_vel, vz_error, 'g-', 'DisplayName', 'Vz誤差');
    
    legend('Location', 'best');
end

% 加速度推定
subplot(2, 2, 3);
hold on;
title('加速度推定');
xlabel('時間 [s]');
ylabel('加速度 [m/s²]');
grid on;

if ~isempty(t_vel)
    ax_est = [];
    ay_est = [];
    az_est = [];
    
    for i = 1:length(vel_results)
        if ~isempty(vel_results{i})
            ax_est = [ax_est; vel_results{i}.acceleration(1)];
            ay_est = [ay_est; vel_results{i}.acceleration(2)];
            az_est = [az_est; vel_results{i}.acceleration(3)];
        end
    end
    
    plot(t_vel, ax_est, 'b-', 'DisplayName', 'Ax推定');
    plot(t_vel, ay_est, 'r-', 'DisplayName', 'Ay推定');
    plot(t_vel, az_est, 'g-', 'DisplayName', 'Az推定');
    
    % センサ値と比較
    plot(truth_data.t, sensor_data.accel3(:, 1), 'b:', 'DisplayName', 'Axセンサ');
    plot(truth_data.t, sensor_data.accel3(:, 2), 'r:', 'DisplayName', 'Ayセンサ');
    plot(truth_data.t, sensor_data.accel3(:, 3), 'g:', 'DisplayName', 'Azセンサ');
    
    legend('Location', 'best');
end

% イノベーション
subplot(2, 2, 4);
hold on;
title('速度推定イノベーション');
xlabel('時間 [s]');
ylabel('イノベーション');
grid on;

if ~isempty(t_vel)
    innov_norm = [];
    for i = 1:length(vel_results)
        if ~isempty(vel_results{i}) && isfield(vel_results{i}, 'innovation')
            innov_norm = [innov_norm; norm(vel_results{i}.innovation)];
        end
    end
    
    if ~isempty(innov_norm)
        plot(t_vel(1:length(innov_norm)), innov_norm, 'b-', 'LineWidth', 1);
    end
end

% 2. 姿勢推定結果の詳細プロット
figure('Name', '姿勢推定詳細結果', 'Position', [150, 550, 1200, 800]);

% 姿勢角時系列
subplot(2, 2, 1);
hold on;
title('姿勢角時系列');
xlabel('時間 [s]');
ylabel('角度 [rad]');
grid on;

% 真値
plot(truth_data.t, truth_data.theta, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Yaw真値');

% 推定値
t_att = [];
roll_est = [];
pitch_est = [];
yaw_est = [];

for i = 1:length(att_results)
    if ~isempty(att_results{i})
        t_att = [t_att; att_results{i}.timestamp];
        roll_est = [roll_est; att_results{i}.attitude(1)];
        pitch_est = [pitch_est; att_results{i}.attitude(2)];
        yaw_est = [yaw_est; att_results{i}.attitude(3)];
    end
end

if ~isempty(t_att)
    plot(t_att, roll_est, 'b-', 'DisplayName', 'Roll推定');
    plot(t_att, pitch_est, 'r-', 'DisplayName', 'Pitch推定');
    plot(t_att, yaw_est, 'g-', 'DisplayName', 'Yaw推定');
end

legend('Location', 'best');

% 角速度推定
subplot(2, 2, 2);
hold on;
title('角速度推定');
xlabel('時間 [s]');
ylabel('角速度 [rad/s]');
grid on;

if ~isempty(t_att)
    wx_est = [];
    wy_est = [];
    wz_est = [];
    
    for i = 1:length(att_results)
        if ~isempty(att_results{i})
            wx_est = [wx_est; att_results{i}.angular_velocity(1)];
            wy_est = [wy_est; att_results{i}.angular_velocity(2)];
            wz_est = [wz_est; att_results{i}.angular_velocity(3)];
        end
    end
    
    plot(t_att, wx_est, 'b-', 'DisplayName', 'Wx推定');
    plot(t_att, wy_est, 'r-', 'DisplayName', 'Wy推定');
    plot(t_att, wz_est, 'g-', 'DisplayName', 'Wz推定');
    
    % センサ値
    t_gyro = truth_data.t(1:4:end);
    plot(t_gyro, sensor_data.gyro3(1:4:end, 1), 'b:', 'DisplayName', 'Wxセンサ');
    plot(t_gyro, sensor_data.gyro3(1:4:end, 2), 'r:', 'DisplayName', 'Wyセンサ');
    plot(t_gyro, sensor_data.gyro3(1:4:end, 3), 'g:', 'DisplayName', 'Wzセンサ');
    
    legend('Location', 'best');
end

% 3. 位置推定結果の詳細プロット
figure('Name', '位置推定詳細結果', 'Position', [200, 500, 1200, 800]);

% 3D軌跡
subplot(2, 2, 1);
hold on;
title('3次元軌跡');
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
grid on;

% 真値
plot3(truth_data.pos(:, 1), truth_data.pos(:, 2), truth_data.pos(:, 3), 'k-', 'LineWidth', 2, 'DisplayName', '真値');

% 推定値
pos_est_x = [];
pos_est_y = [];
pos_est_z = [];

for i = 1:length(pos_results)
    if ~isempty(pos_results{i})
        pos_est_x = [pos_est_x; pos_results{i}.position(1)];
        pos_est_y = [pos_est_y; pos_results{i}.position(2)];
        pos_est_z = [pos_est_z; pos_results{i}.position(3)];
    end
end

if ~isempty(pos_est_x)
    plot3(pos_est_x, pos_est_y, pos_est_z, 'b-', 'LineWidth', 1.5, 'DisplayName', '推定値');
end

legend('Location', 'best');
view(3);

% 位置誤差統計
subplot(2, 2, 2);
if ~isempty(pos_est_x)
    pos_errors = [];
    for i = 1:length(pos_results)
        if ~isempty(pos_results{i})
            idx = i * 40;
            if idx <= size(truth_data.pos, 1)
                error_vec = pos_results{i}.position - truth_data.pos(idx, :)';
                pos_errors = [pos_errors; norm(error_vec)];
            end
        end
    end
    
    if ~isempty(pos_errors)
        histogram(pos_errors, 20);
        title(sprintf('位置誤差分布 (平均: %.2f m)', mean(pos_errors)));
        xlabel('誤差 [m]');
        ylabel('頻度');
        grid on;
    end
end

end