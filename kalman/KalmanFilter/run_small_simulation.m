function run_small_simulation()
% RUN_SMALL_SIMULATION - 小規模カスケードUKFシミュレーション
% データ量を制限してテスト実行

% パラメータ定義
VEL_FACTOR = 1;    % 速度推定頻度（毎回）
ATT_FACTOR = 4;    % 姿勢推定頻度（4回に1回）
POS_FACTOR = 40;   % 位置推定頻度（40回に1回）

% データ制限（最初の1000ポイントのみ）
MAX_DATA_POINTS = 1000;

fprintf('=== 小規模カスケードUKFシミュレーション開始 ===\n');

% パス設定
current_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(current_dir, '..', 'GenerateData'));

% CSVデータ読み込み
csv_file = fullfile(current_dir, '..', '..', 'sim_data.csv');
[sensor_data_full, truth_data_full] = read_csv_data(csv_file);

% データを制限
sensor_data = struct();
truth_data = struct();

fields = fieldnames(sensor_data_full);
for i = 1:length(fields)
    field = fields{i};
    data = sensor_data_full.(field);
    if size(data, 1) >= MAX_DATA_POINTS
        sensor_data.(field) = data(1:MAX_DATA_POINTS, :);
    else
        sensor_data.(field) = data;
    end
end

fields = fieldnames(truth_data_full);
for i = 1:length(fields)
    field = fields{i};
    data = truth_data_full.(field);
    if size(data, 1) >= MAX_DATA_POINTS
        truth_data.(field) = data(1:MAX_DATA_POINTS, :);
    else
        truth_data.(field) = data;
    end
end

fprintf('データ制限完了: %d ポイント\n', length(sensor_data.t));

% パラメータ設定
params = config_params();

% UKFインスタンス作成
ukf_vel = UKF_Calculator(6, 3);  % 速度推定: 6状態, 3観測
ukf_att = UKF_Calculator(6, 6);  % 姿勢推定: 6状態, 6観測  
ukf_pos = UKF_Calculator(6, 6);  % 位置推定: 6状態, 6観測

% 結果保存用
vel_results = cell(length(sensor_data.t), 1);
att_results = cell(ceil(length(sensor_data.t) / ATT_FACTOR), 1);
pos_results = cell(ceil(length(sensor_data.t) / POS_FACTOR), 1);

% 前回状態
prev_vel_state = [];
prev_att_state = [];
prev_pos_state = [];

% 推定値バッファ
vel_buffer = cell(ATT_FACTOR, 1);
att_buffer = cell(POS_FACTOR / ATT_FACTOR, 1);

% メインループ
att_idx = 1;
pos_idx = 1;
vel_buf_idx = 1;
att_buf_idx = 1;

fprintf('データ処理開始...\n');
for i = 1:length(sensor_data.t)
    if mod(i, 100) == 0
        fprintf('処理中: %d/%d (%.1f%%)\n', i, length(sensor_data.t), i/length(sensor_data.t)*100);
    end
    
    % 速度推定（毎回実行）
    if mod(i-1, VEL_FACTOR) == 0
        vel_results{i} = velocity_estimator(sensor_data, params, ukf_vel, i, prev_vel_state);
        prev_vel_state = vel_results{i};
        
        % 速度推定結果をバッファに保存
        vel_buffer{vel_buf_idx} = vel_results{i};
        vel_buf_idx = vel_buf_idx + 1;
        if vel_buf_idx > ATT_FACTOR
            vel_buf_idx = 1;
        end
    end
    
    % 姿勢推定（ATT_FACTOR回に1回実行）
    if mod(i-1, ATT_FACTOR) == 0
        % 速度推定結果の平均を計算
        vel_avg = calculate_velocity_average(vel_buffer);
        
        att_results{att_idx} = attitude_estimator(sensor_data, params, ukf_att, i, prev_att_state, vel_avg);
        prev_att_state = att_results{att_idx};
        
        % 姿勢推定結果をバッファに保存
        att_buffer{att_buf_idx} = att_results{att_idx};
        att_buf_idx = att_buf_idx + 1;
        if att_buf_idx > (POS_FACTOR / ATT_FACTOR)
            att_buf_idx = 1;
        end
        
        att_idx = att_idx + 1;
    end
    
    % 位置推定（POS_FACTOR回に1回実行）
    if mod(i-1, POS_FACTOR) == 0
        % 速度と姿勢推定結果の平均を計算
        vel_avg = calculate_velocity_average(vel_buffer);
        att_avg = calculate_attitude_average(att_buffer);
        
        pos_results{pos_idx} = position_estimator(sensor_data, params, ukf_pos, i, prev_pos_state, {vel_avg}, {att_avg});
        prev_pos_state = pos_results{pos_idx};
        
        fprintf('位置推定 %d: [%.2f, %.2f, %.2f] m\n', pos_idx, pos_results{pos_idx}.position);
        
        pos_idx = pos_idx + 1;
    end
end

fprintf('小規模シミュレーション完了!\n');

% 結果統計
fprintf('\n=== 結果統計 ===\n');
vel_count = sum(~cellfun(@isempty, vel_results));
att_count = sum(~cellfun(@isempty, att_results));
pos_count = sum(~cellfun(@isempty, pos_results));

fprintf('速度推定回数: %d\n', vel_count);
fprintf('姿勢推定回数: %d\n', att_count);
fprintf('位置推定回数: %d\n', pos_count);

% 簡易プロット
try
    create_simple_plot(vel_results, att_results, pos_results, truth_data);
catch ME
    fprintf('プロット作成エラー: %s\n', ME.message);
end

end

function vel_avg = calculate_velocity_average(vel_buffer)
% 速度推定結果の平均を計算
vel_sum = zeros(3, 1, 'single');
count = 0;
for i = 1:length(vel_buffer)
    if ~isempty(vel_buffer{i})
        vel_sum = vel_sum + vel_buffer{i}.velocity;
        count = count + 1;
    end
end
if count > 0
    vel_avg = struct();
    vel_avg.velocity = vel_sum / count;
    vel_avg.acceleration = zeros(3, 1, 'single');
else
    vel_avg = [];
end
end

function att_avg = calculate_attitude_average(att_buffer)
% 姿勢推定結果の平均を計算
att_sum = zeros(3, 1, 'single');
count = 0;
for i = 1:length(att_buffer)
    if ~isempty(att_buffer{i})
        att_sum = att_sum + att_buffer{i}.attitude;
        count = count + 1;
    end
end
if count > 0
    att_avg = struct();
    att_avg.attitude = att_sum / count;
else
    att_avg = [];
end
end

function create_simple_plot(vel_results, att_results, pos_results, truth_data)
% 簡単なプロット作成

figure('Name', '小規模シミュレーション結果', 'Position', [100, 100, 1200, 800]);

% 位置軌跡
subplot(2, 2, 1);
hold on;
title('位置軌跡 (XY)');
xlabel('X [m]');
ylabel('Y [m]');
grid on;
axis equal;

% 真値
plot(truth_data.pos(:, 1), truth_data.pos(:, 2), 'k-', 'LineWidth', 2, 'DisplayName', '真値');

% 推定値
pos_est = [];
for i = 1:length(pos_results)
    if ~isempty(pos_results{i})
        pos_est = [pos_est; pos_results{i}.position(1:2)'];
    end
end

if ~isempty(pos_est)
    plot(pos_est(:, 1), pos_est(:, 2), 'b-', 'LineWidth', 1.5, 'DisplayName', '推定値');
end

legend('Location', 'best');

% 速度推定誤差
subplot(2, 2, 2);
hold on;
title('速度推定誤差');
xlabel('時間 [s]');
ylabel('誤差 [m/s]');
grid on;

vel_error = [];
t_vel = [];
for i = 1:length(vel_results)
    if ~isempty(vel_results{i}) && i <= length(truth_data.vel)
        true_vel = truth_data.vel(i, 1:3)';
        est_vel = vel_results{i}.velocity;
        vel_error = [vel_error; norm(est_vel - true_vel)];
        t_vel = [t_vel; truth_data.t(i)];
    end
end

if ~isempty(vel_error)
    plot(t_vel, vel_error, 'b-', 'LineWidth', 1);
    fprintf('平均速度誤差: %.4f m/s\n', mean(vel_error));
end

% 位置推定誤差
subplot(2, 2, 3);
hold on;
title('位置推定誤差');
xlabel('時間 [s]');
ylabel('誤差 [m]');
grid on;

pos_error = [];
t_pos = [];
for i = 1:length(pos_results)
    if ~isempty(pos_results{i})
        idx = i * 40;
        if idx <= size(truth_data.pos, 1)
            true_pos = truth_data.pos(idx, :)';
            est_pos = pos_results{i}.position;
            pos_error = [pos_error; norm(est_pos - true_pos)];
            t_pos = [t_pos; truth_data.t(idx)];
        end
    end
end

if ~isempty(pos_error)
    plot(t_pos, pos_error, 'r-', 'LineWidth', 1);
    fprintf('平均位置誤差: %.4f m\n', mean(pos_error));
end

% 姿勢推定 (Yaw)
subplot(2, 2, 4);
hold on;
title('Yaw角推定');
xlabel('時間 [s]');
ylabel('Yaw [rad]');
grid on;

% 真値
plot(truth_data.t, truth_data.theta, 'k-', 'LineWidth', 2, 'DisplayName', '真値');

% 推定値
yaw_est = [];
t_att = [];
for i = 1:length(att_results)
    if ~isempty(att_results{i})
        yaw_est = [yaw_est; att_results{i}.attitude(3)];
        t_att = [t_att; att_results{i}.timestamp];
    end
end

if ~isempty(yaw_est)
    plot(t_att, yaw_est, 'b-', 'LineWidth', 1.5, 'DisplayName', '推定値');
end

legend('Location', 'best');

end